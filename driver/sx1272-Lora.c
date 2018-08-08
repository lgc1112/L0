/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1272-LoRa.c
 * \brief      SX1272 RF chip driver mode LoRa
 *
 * \version    2.0.0 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */

#include "sx1272-LoRa.h"

//#include "platform.h"

//#include "radio.h"

//#include "sx1272-Hal.h"
//#include "sx1272.h"

//#include "sx1272-LoRaMisc.h"
//#include "sx1272-LoRa.h"

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET                                 -139.0

/*!
 * Frequency hopping frequencies table
 */
const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

// Default settings
tLoRaSettings LoRaSettings =
{
    870000000,        // RFFrequency
    20,               // Power
    2,                // SignalBw [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] 
    7,                // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    2,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    true,             // CrcOn [0: OFF, 1: ON]
    false,            // ImplicitHeaderOn [0: OFF, 1: ON]
    1,                // RxSingleOn [0: Continuous, 1 Single]
    0,                // FreqHopOn [0: OFF, 1: ON]
    4,                // HopPeriod Hops every frequency hopping period symbols
    100,              // TxPacketTimeout
    100,              // RxPacketTimeout
    128,              // PayloadLength (used for implicit header mode)
};

/*!
 * SX1272 LoRa registers variable
 */
tSX1272LR* SX1272LR;

/*!
 * Local RF buffer for communication support
 */
static uint8_t RFBuffer[RF_BUFFER_SIZE];

/*!
 * RF state machine variable
 */
static uint8_t RFLRState = RFLR_STATE_IDLE;

/*!
 * Rx management support variables
 */
static uint16_t RxPacketSize = 0;
static int8_t RxPacketSnrEstimate;
static double RxPacketRssiValue;
static uint8_t RxGain = 1;
static uint32_t RxTimeoutTimer = 0;
/*!
 * PacketTimeout Stores the Rx window time value for packet reception
 */
static uint32_t PacketTimeout;

/*!
 * Tx management support variables
 */
static uint16_t TxPacketSize = 0;

SPI_HandleTypeDef * g_SpiHandle;

extern void LoraRegWrite(SPI_HandleTypeDef *SpiHandle,uint8_t addr, uint8_t *buffer, uint8_t size);
extern void LoraRegRead(SPI_HandleTypeDef *SpiHandle, byte adr, char* pData, byte read_len);

////////////////////////////////////
void SX1272Write( uint8_t addr, uint8_t data )
{
    //SX1272WriteBuffer( addr, &data, 1 );
    LoraRegWrite(g_SpiHandle,addr,&data,1);
}
void SX1272Read( uint8_t addr, uint8_t *data )
{
    //SX1272ReadBuffer( addr, data, 1 );
    LoraRegRead(g_SpiHandle,addr,(char*)data,1);
}
void SX1272WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    LoraRegWrite(g_SpiHandle,addr,buffer,size);
}
void SX1272ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    LoraRegRead(g_SpiHandle,addr,(char*)buffer,size);
}
void SX1272LoRaSetRFFrequency( uint32_t freq )
{
    LoRaSettings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1272LR->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1272LR->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1272LR->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    SX1272WriteBuffer( REG_LR_FRFMSB, &SX1272LR->RegFrfMsb, 3 );
}
void SX1272LoRaSetPa20dBm( bool enale )
{
    SX1272Read( REG_LR_PADAC, &SX1272LR->RegPaDac );
    
    if( enale == true )
    {
        SX1272LR->RegPaDac = 0x87;
    }
    else
    {
        SX1272LR->RegPaDac = 0x84;
    }
    SX1272Write( REG_LR_PADAC, SX1272LR->RegPaDac );
}
void SX1272LoRaSetRFPower( int8_t power )
{
    SX1272Read( REG_LR_PACONFIG, &SX1272LR->RegPaConfig );
    SX1272Read( REG_LR_PADAC, &SX1272LR->RegPaDac );
    
    if( ( SX1272LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1272LR->RegPaDac & 0x07 ) == 0x07 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1272LR->RegPaConfig = ( SX1272LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            SX1272LR->RegPaConfig = ( SX1272LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        SX1272LR->RegPaConfig = ( SX1272LR->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1272Write( REG_LR_PACONFIG, SX1272LR->RegPaConfig );
    LoRaSettings.Power = power;
}
void SX1272LoRaSetNbTrigPeaks( uint8_t value )
{
    SX1272Read( 0x31, &SX1272LR->RegDetectOptimize );
    SX1272LR->RegDetectOptimize = ( SX1272LR->RegDetectOptimize & 0xF8 ) | value;
    SX1272Write( 0x31, SX1272LR->RegDetectOptimize );
}
void SX1272LoRaSetSpreadingFactor( uint8_t factor )
{

    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }
    
    if( factor == 6 )
    {
        SX1272LoRaSetNbTrigPeaks( 5 );
    }
    else
    {
        SX1272LoRaSetNbTrigPeaks( 3 );
    }

    SX1272Read( REG_LR_MODEMCONFIG2, &SX1272LR->RegModemConfig2 );    
    SX1272LR->RegModemConfig2 = ( SX1272LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    SX1272Write( REG_LR_MODEMCONFIG2, SX1272LR->RegModemConfig2 );    
    LoRaSettings.SpreadingFactor = factor;
}

void SX1272LoRaSetErrorCoding( uint8_t value )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    SX1272LR->RegModemConfig1 = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 3 );
    SX1272Write( REG_LR_MODEMCONFIG1, SX1272LR->RegModemConfig1 );
    LoRaSettings.ErrorCoding = value;
}
void SX1272LoRaSetPacketCrcOn( bool enable )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    SX1272LR->RegModemConfig1 = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK ) | ( enable << 1 );
    SX1272Write( REG_LR_MODEMCONFIG1, SX1272LR->RegModemConfig1 );
    LoRaSettings.CrcOn = enable;
}

void SX1272LoRaSetSignalBandwidth( uint8_t bw )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    SX1272LR->RegModemConfig1 = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 6 );
    SX1272Write( REG_LR_MODEMCONFIG1, SX1272LR->RegModemConfig1 );
    LoRaSettings.SignalBw = bw;
}

void SX1272LoRaSetImplicitHeaderOn( bool enable )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    SX1272LR->RegModemConfig1 = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable << 2 );
    SX1272Write( REG_LR_MODEMCONFIG1, SX1272LR->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = enable;
}
void SX1272LoRaSetSymbTimeout( uint16_t value )
{
    SX1272ReadBuffer( REG_LR_MODEMCONFIG2, &SX1272LR->RegModemConfig2, 2 );

    SX1272LR->RegModemConfig2 = ( SX1272LR->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1272LR->RegSymbTimeoutLsb = value & 0xFF;
    SX1272WriteBuffer( REG_LR_MODEMCONFIG2, &SX1272LR->RegModemConfig2, 2 );
}
void SX1272LoRaSetPayloadLength( uint8_t value )
{
    SX1272LR->RegPayloadLength = value;
    SX1272Write( REG_LR_PAYLOADLENGTH, SX1272LR->RegPayloadLength );
    LoRaSettings.PayloadLength = value;
}
void SX1272LoRaSetLowDatarateOptimize( bool enable )
{
    SX1272Read( REG_LR_MODEMCONFIG1, &SX1272LR->RegModemConfig1 );
    SX1272LR->RegModemConfig1 = ( SX1272LR->RegModemConfig1 & RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) | enable;
    SX1272Write( REG_LR_MODEMCONFIG1, SX1272LR->RegModemConfig1 );
}
void SX1272ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1272ReadBuffer( 0, buffer, size );
}
void SX1272WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1272WriteBuffer( 0, buffer, size );
}

////////////////////////////////////
void SX1272LoRaInit( SPI_HandleTypeDef * SpiHandle )
{
	g_SpiHandle = SpiHandle;
	
    RFLRState = RFLR_STATE_IDLE;

    SX1272LoRaSetDefaults( );
    
    //SX1272ReadBuffer( REG_LR_OPMODE, SX1272Regs + 1, 0x70 - 1 );
	LoraRegRead(SpiHandle,REG_LR_OPMODE,(char*)(SX1272Regs + 1),0x70 - 1 );

    SX1272LR->RegPaConfig = ( SX1272LR->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) |
                            RFLR_PACONFIG_PASELECT_PABOOST;

    SX1272LR->RegLna = RFLR_LNA_GAIN_G1 | RFLR_LNA_BOOST_ON;

    //SX1272WriteBuffer( REG_LR_OPMODE, SX1272Regs + 1, 0x70 - 1 );
	LoraRegWrite(SpiHandle,REG_LR_OPMODE,SX1272Regs + 1,0x70 - 1);

    // set the RF settings 
    SX1272LoRaSetRFFrequency( LoRaSettings.RFFrequency );
    SX1272LoRaSetPa20dBm( true );
    SX1272LoRaSetRFPower( LoRaSettings.Power );
    SX1272LoRaSetSpreadingFactor( LoRaSettings.SpreadingFactor ); // SF6 only operates in implicit header mode.
    SX1272LoRaSetErrorCoding( LoRaSettings.ErrorCoding );
    SX1272LoRaSetPacketCrcOn( LoRaSettings.CrcOn );
    SX1272LoRaSetSignalBandwidth( LoRaSettings.SignalBw );
    
    SX1272LoRaSetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn );
    SX1272LoRaSetSymbTimeout( 0x3FF );
    SX1272LoRaSetPayloadLength( LoRaSettings.PayloadLength );
    SX1272LoRaSetLowDatarateOptimize( true );

    SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );
}

void SX1272LoRaSetDefaults( void )
{
    // REMARK: See SX1272 datasheet for modified default values.

    // Sets IF frequency selection manual
    SX1272LR->RegDetectOptimize = 0x43; // default value 0xC3
    SX1272Write( 0x31, SX1272LR->RegDetectOptimize );

    SX1272Read( REG_LR_VERSION, &SX1272LR->RegVersion );
}

#if 0
void SX1272LoRaReset( void )
{
    uint32_t startTick;
    
    SX1272SetReset( RADIO_RESET_ON );
    
    // Wait 1ms
    startTick = GET_TICK_COUNT( );
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 1 ) );    

    SX1272SetReset( RADIO_RESET_OFF );
    
    // Wait 6ms
    startTick = GET_TICK_COUNT( );
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 6 ) );    
}
#endif

void SX1272LoRaSetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;

    opModePrev = SX1272LR->RegOpMode & ~RFLR_OPMODE_MASK;

    if( opMode != opModePrev )
    {
        if( opMode == RFLR_OPMODE_TRANSMITTER )
        {
            antennaSwitchTxOn = true;
        }
        else
        {
            antennaSwitchTxOn = false;
        }
        if( antennaSwitchTxOn != antennaSwitchTxOnPrev )
        {
            antennaSwitchTxOnPrev = antennaSwitchTxOn;
            RXTX( antennaSwitchTxOn ); // Antenna switch control
        }
        SX1272LR->RegOpMode = ( SX1272LR->RegOpMode & RFLR_OPMODE_MASK ) | opMode;

        SX1272Write( REG_LR_OPMODE, SX1272LR->RegOpMode );        
    }
}

uint8_t SX1272LoRaGetOpMode( void )
{
    SX1272Read( REG_LR_OPMODE, &SX1272LR->RegOpMode );
    
    return SX1272LR->RegOpMode & ~RFLR_OPMODE_MASK;
}

uint8_t SX1272LoRaReadRxGain( void )
{
    SX1272Read( REG_LR_LNA, &SX1272LR->RegLna );
    return( SX1272LR->RegLna >> 5 ) & 0x07;
}

double SX1272LoRaReadRssi( void )
{
    // Reads the RSSI value
    SX1272Read( REG_LR_RSSIVALUE, &SX1272LR->RegRssiValue );

    return RSSI_OFFSET + ( double )SX1272LR->RegRssiValue;
}

uint8_t SX1272LoRaGetPacketRxGain( void )
{
    return RxGain;
}

int8_t SX1272LoRaGetPacketSnr( void )
{
    return RxPacketSnrEstimate;
}

double SX1272LoRaGetPacketRssi( void )
{
    return RxPacketRssiValue;
}

void SX1272LoRaStartRx( void )
{
    SX1272LoRaSetRFState( RFLR_STATE_RX_INIT );
}

void SX1272LoRaGetRxPacket( void *buffer, uint16_t *size )
{
    *size = RxPacketSize;
    RxPacketSize = 0;
    memcpy( ( void * )buffer, ( void * )RFBuffer, ( size_t )*size );
}

void SX1272LoRaSetTxPacket( const void *buffer, uint16_t size )
{
    TxPacketSize = size;
    memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize ); 

    RFLRState = RFLR_STATE_TX_INIT;
}

uint8_t SX1272LoRaGetRFState( void )
{
    return RFLRState;
}

void SX1272LoRaSetRFState( uint8_t state )
{
    RFLRState = state;
}

/*!
 * \brief Process the LoRa modem Rx and Tx state machines depending on the
 *        SX1272 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint32_t SX1272LoRaProcess( void )
{
    uint32_t result = RF_BUSY;
    
    switch( RFLRState )
    {
    case RFLR_STATE_IDLE:
        break;
    case RFLR_STATE_RX_INIT:
        
        SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        SX1272LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    //RFLR_IRQFLAGS_RXDONE |
                                    //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1272Write( REG_LR_IRQFLAGSMASK, SX1272LR->RegIrqFlagsMask );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1272LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1272Read( REG_LR_HOPCHANNEL, &SX1272LR->RegHopChannel );
            SX1272LoRaSetRFFrequency( HoppingFrequencies[SX1272LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1272LR->RegHopPeriod = 255;
        }
        
        SX1272Write( REG_LR_HOPPERIOD, SX1272LR->RegHopPeriod );
                
                                    // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
        SX1272LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CadDetected               ModeReady
        SX1272LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1272WriteBuffer( REG_LR_DIOMAPPING1, &SX1272LR->RegDioMapping1, 2 );
    
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {

            SX1272LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
        else // Rx continuous mode
        {
            SX1272LR->RegFifoAddrPtr = SX1272LR->RegFifoRxBaseAddr;
            SX1272Write( REG_LR_FIFOADDRPTR, SX1272LR->RegFifoAddrPtr );
            
            SX1272LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
        }
        
        memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );

        PacketTimeout = LoRaSettings.RxPacketTimeout;
        RxTimeoutTimer = GET_TICK_COUNT( );
        RFLRState = RFLR_STATE_RX_RUNNING;
        break;
    case RFLR_STATE_RX_RUNNING:
        
        if( DIO0 == 1 ) // RxDone
        {
            RxTimeoutTimer = GET_TICK_COUNT( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1272Read( REG_LR_HOPCHANNEL, &SX1272LR->RegHopChannel );
                SX1272LoRaSetRFFrequency( HoppingFrequencies[SX1272LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
            RFLRState = RFLR_STATE_RX_DONE;
        }
        if( DIO2 == 1 ) // FHSS Changed Channel
        {
            RxTimeoutTimer = GET_TICK_COUNT( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1272Read( REG_LR_HOPCHANNEL, &SX1272LR->RegHopChannel );
                SX1272LoRaSetRFFrequency( HoppingFrequencies[SX1272LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
            // Debug
            RxGain = SX1272LoRaReadRxGain( );
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            if( ( GET_TICK_COUNT( ) - RxTimeoutTimer ) > PacketTimeout )
            {
                RFLRState = RFLR_STATE_RX_TIMEOUT;
            }
        }
        break;
    case RFLR_STATE_RX_DONE:
        SX1272Read( REG_LR_IRQFLAGS, &SX1272LR->RegIrqFlags );
        if( ( SX1272LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
        {
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );
            
            if( LoRaSettings.RxSingleOn == true ) // Rx single mode
            {
                RFLRState = RFLR_STATE_RX_INIT;
            }
            else
            {
                RFLRState = RFLR_STATE_RX_RUNNING;
            }
            break;
        }
        
        {
            uint8_t rxSnrEstimate;
            SX1272Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
            if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
                RxPacketSnrEstimate = -RxPacketSnrEstimate;
            }
            else
            {
                // Divide by 4
                RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
            }
        }
    
        SX1272Read( REG_LR_PKTRSSIVALUE, &SX1272LR->RegPktRssiValue );
    
        if( RxPacketSnrEstimate < 0 )
        {
            RxPacketRssiValue = RSSI_OFFSET + ( ( double )SX1272LR->RegPktRssiValue ) + RxPacketSnrEstimate;
        }
        else
        {
            RxPacketRssiValue = RSSI_OFFSET + ( 1.0666 * ( ( double )SX1272LR->RegPktRssiValue ) );
        }
    
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            SX1272LR->RegFifoAddrPtr = SX1272LR->RegFifoRxBaseAddr;
            SX1272Write( REG_LR_FIFOADDRPTR, SX1272LR->RegFifoAddrPtr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1272LR->RegPayloadLength;
                SX1272ReadFifo( RFBuffer, SX1272LR->RegPayloadLength );
            }
            else
            {
                SX1272Read( REG_LR_NBRXBYTES, &SX1272LR->RegNbRxBytes );
                RxPacketSize = SX1272LR->RegNbRxBytes;
                SX1272ReadFifo( RFBuffer, SX1272LR->RegNbRxBytes );
            }
        }
        else // Rx continuous mode
        {
            SX1272Read( REG_LR_FIFORXCURRENTADDR, &SX1272LR->RegFifoRxCurrentAddr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1272LR->RegPayloadLength;
                SX1272LR->RegFifoAddrPtr = SX1272LR->RegFifoRxCurrentAddr;
                SX1272Write( REG_LR_FIFOADDRPTR, SX1272LR->RegFifoAddrPtr );
                SX1272ReadFifo( RFBuffer, SX1272LR->RegPayloadLength );
            }
            else
            {
                SX1272Read( REG_LR_NBRXBYTES, &SX1272LR->RegNbRxBytes );
                RxPacketSize = SX1272LR->RegNbRxBytes;
                SX1272LR->RegFifoAddrPtr = SX1272LR->RegFifoRxCurrentAddr;
                SX1272Write( REG_LR_FIFOADDRPTR, SX1272LR->RegFifoAddrPtr );
                SX1272ReadFifo( RFBuffer, SX1272LR->RegNbRxBytes );
            }
        }
        
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            RFLRState = RFLR_STATE_RX_INIT;
        }
        else // Rx continuous mode
        {
            RFLRState = RFLR_STATE_RX_RUNNING;
        }
        result = RF_RX_DONE;
        break;
    case RFLR_STATE_RX_TIMEOUT:
        RFLRState = RFLR_STATE_RX_INIT;
        result = RF_RX_TIMEOUT;
        break;
    case RFLR_STATE_TX_INIT:

        SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1272LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1272LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1272Read( REG_LR_HOPCHANNEL, &SX1272LR->RegHopChannel );
            SX1272LoRaSetRFFrequency( HoppingFrequencies[SX1272LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1272LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1272LR->RegHopPeriod = 0;
        }
        SX1272Write( REG_LR_HOPPERIOD, SX1272LR->RegHopPeriod );
        SX1272Write( REG_LR_IRQFLAGSMASK, SX1272LR->RegIrqFlagsMask );

        // Initializes the payload size
        SX1272LR->RegPayloadLength = TxPacketSize;
        SX1272Write( REG_LR_PAYLOADLENGTH, SX1272LR->RegPayloadLength );
        
        SX1272LR->RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
        SX1272Write( REG_LR_FIFOTXBASEADDR, SX1272LR->RegFifoTxBaseAddr );

        SX1272LR->RegFifoAddrPtr = SX1272LR->RegFifoTxBaseAddr;
        SX1272Write( REG_LR_FIFOADDRPTR, SX1272LR->RegFifoAddrPtr );
        
        // Write payload buffer to LORA modem
        SX1272WriteFifo( RFBuffer, SX1272LR->RegPayloadLength );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
        SX1272LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
        SX1272LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
        SX1272WriteBuffer( REG_LR_DIOMAPPING1, &SX1272LR->RegDioMapping1, 2 );

        SX1272LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );

        RFLRState = RFLR_STATE_TX_RUNNING;
        break;
    case RFLR_STATE_TX_RUNNING:
        if( DIO0 == 1 ) // TxDone
        {
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
            RFLRState = RFLR_STATE_TX_DONE;   
        }
        if( DIO2 == 1 ) // FHSS Changed Channel
        {
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1272Read( REG_LR_HOPCHANNEL, &SX1272LR->RegHopChannel );
                SX1272LoRaSetRFFrequency( HoppingFrequencies[SX1272LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
        }
        break;
    case RFLR_STATE_TX_DONE:
        // optimize the power consumption by switching off the transmitter as soon as the packet has been sent
        SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        RFLRState = RFLR_STATE_IDLE;
        result = RF_TX_DONE;
        break;
    case RFLR_STATE_CAD_INIT:    
        SX1272LoRaSetOpMode( RFLR_OPMODE_STANDBY );
    
        SX1272LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    //RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                                    //RFLR_IRQFLAGS_CADDETECTED;
        SX1272Write( REG_LR_IRQFLAGSMASK, SX1272LR->RegIrqFlagsMask );
           
                                    // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
        SX1272LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CAD Detected              ModeReady
        SX1272LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1272WriteBuffer( REG_LR_DIOMAPPING1, &SX1272LR->RegDioMapping1, 2 );
            
        SX1272LoRaSetOpMode( RFLR_OPMODE_CAD );
        RFLRState = RFLR_STATE_CAD_RUNNING;
        break;
    case RFLR_STATE_CAD_RUNNING:
        if( DIO3 == 1 ) //CAD Done interrupt
        { 
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE  );
            if( DIO4 == 1 ) // CAD Detected interrupt
            {
                // Clear Irq
                SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED  );
                // CAD detected, we have a LoRa preamble
                RFLRState = RFLR_STATE_RX_INIT;
                result = RF_CHANNEL_ACTIVITY_DETECTED;
            } 
            else
            {    
                // The device goes in Standby Mode automatically    
                RFLRState = RFLR_STATE_IDLE;
                result = RF_CHANNEL_EMPTY;
            }
        }   
        break;
    
    default:
        break;
    } 
    return result;
}


