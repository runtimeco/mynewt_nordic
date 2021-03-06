/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**
 * @defgroup nrf_spis_hal SPIS HAL
 * @{
 * @ingroup nrf_spis
 *
 * @brief Hardware access layer for accessing the SPIS peripheral.
 */

#ifndef NRF_SPIS_H__
#define NRF_SPIS_H__

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief This value can be used as a parameter for the @ref nrf_spis_pins_set
 *        function to specify that a given SPI signal (SCK, MOSI, or MISO)
 *        shall not be connected to a physical pin.
 */
#define NRF_SPIS_PIN_NOT_CONNECTED  0xFFFFFFFF


/**
 * @brief SPIS tasks.
 */
typedef enum
{
    /*lint -save -e30*/
    NRF_SPIS_TASK_ACQUIRE = offsetof(NRF_SPIS_Type, TASKS_ACQUIRE), ///< Acquire SPI semaphore.
    NRF_SPIS_TASK_RELEASE = offsetof(NRF_SPIS_Type, TASKS_RELEASE), ///< Release SPI semaphore, enabling the SPI slave to acquire it.
    /*lint -restore*/
} nrf_spis_task_t;

/**
 * @brief SPIS events.
 */
typedef enum
{
    /*lint -save -e30*/
    NRF_SPIS_EVENT_END      = offsetof(NRF_SPIS_Type, EVENTS_END),     ///< Granted transaction completed.
    NRF_SPIS_EVENT_ACQUIRED = offsetof(NRF_SPIS_Type, EVENTS_ACQUIRED) ///< Semaphore acquired.
    /*lint -restore*/
} nrf_spis_event_t;

/**
 * @brief SPIS shortcuts.
 */
typedef enum
{
    NRF_SPIS_SHORT_END_ACQUIRE = SPIS_SHORTS_END_ACQUIRE_Msk ///< Shortcut between END event and ACQUIRE task.
} nrf_spis_short_mask_t;

/**
 * @brief SPIS interrupts.
 */
typedef enum
{
    NRF_SPIS_INT_END_MASK      = SPIS_INTENSET_END_Msk,     ///< Interrupt on END event.
    NRF_SPIS_INT_ACQUIRED_MASK = SPIS_INTENSET_ACQUIRED_Msk ///< Interrupt on ACQUIRED event.
} nrf_spis_int_mask_t;

/**
 * @brief SPI modes.
 */
typedef enum
{
    NRF_SPIS_MODE_0, ///< SCK active high, sample on leading edge of clock.
    NRF_SPIS_MODE_1, ///< SCK active high, sample on trailing edge of clock.
    NRF_SPIS_MODE_2, ///< SCK active low, sample on leading edge of clock.
    NRF_SPIS_MODE_3  ///< SCK active low, sample on trailing edge of clock.
} nrf_spis_mode_t;

/**
 * @brief SPI bit orders.
 */
typedef enum
{
    NRF_SPIS_BIT_ORDER_MSB_FIRST = SPIS_CONFIG_ORDER_MsbFirst, ///< Most significant bit shifted out first.
    NRF_SPIS_BIT_ORDER_LSB_FIRST = SPIS_CONFIG_ORDER_LsbFirst  ///< Least significant bit shifted out first.
} nrf_spis_bit_order_t;

/**
 * @brief SPI semaphore status.
 */
typedef enum
{
    NRF_SPIS_SEMSTAT_FREE       = 0, ///< Semaphore is free.
    NRF_SPIS_SEMSTAT_CPU        = 1, ///< Semaphore is assigned to the CPU.
    NRF_SPIS_SEMSTAT_SPIS       = 2, ///< Semaphore is assigned to the SPI slave.
    NRF_SPIS_SEMSTAT_CPUPENDING = 3  ///< Semaphore is assigned to the SPI, but a handover to the CPU is pending.
} nrf_spis_semstat_t;

/**
 * @brief SPIS status.
 */
typedef enum
{
    NRF_SPIS_STATUS_OVERREAD = SPIS_STATUS_OVERREAD_Msk, ///< TX buffer over-read detected and prevented.
    NRF_SPIS_STATUS_OVERFLOW = SPIS_STATUS_OVERFLOW_Msk  ///< RX buffer overflow detected and prevented.
} nrf_spis_status_mask_t;

/**
 * @brief Function for activating a specific SPIS task.
 *
 * @param[in] p_spis    SPIS instance.
 * @param[in] spis_task Task to activate.
 */
__STATIC_INLINE void nrf_spis_task_trigger(NRF_SPIS_Type * p_spis,
                                           nrf_spis_task_t spis_task);

/**
 * @brief Function for getting the address of a specific SPIS task register.
 *
 * @param[in] p_spis    SPIS instance.
 * @param[in] spis_task Requested task.
 *
 * @return Address of the specified task register.
 */
__STATIC_INLINE uint32_t nrf_spis_task_address_get(NRF_SPIS_Type const * p_spis,
                                                   nrf_spis_task_t spis_task);

/**
 * @brief Function for clearing a specific SPIS event.
 *
 * @param[in] p_spis     SPIS instance.
 * @param[in] spis_event Event to clear.
 */
__STATIC_INLINE void nrf_spis_event_clear(NRF_SPIS_Type * p_spis,
                                          nrf_spis_event_t spis_event);

/**
 * @brief Function for checking the state of a specific SPIS event.
 *
 * @param[in] p_spis     SPIS instance.
 * @param[in] spis_event Event to check.
 *
 * @retval true  If the event is set.
 * @retval false If the event is not set.
 */
__STATIC_INLINE bool nrf_spis_event_check(NRF_SPIS_Type const * p_spis,
                                          nrf_spis_event_t spis_event);

/**
 * @brief Function for getting the address of a specific SPIS event register.
 *
 * @param[in] p_spis     SPIS instance.
 * @param[in] spis_event Requested event.
 *
 * @return Address of the specified event register.
 */
__STATIC_INLINE uint32_t nrf_spis_event_address_get(NRF_SPIS_Type const * p_spis,
                                                    nrf_spis_event_t spis_event);

/**
 * @brief Function for enabling specified shortcuts.
 *
 * @param[in] p_spis           SPIS instance.
 * @param[in] spis_shorts_mask Shortcuts to enable.
 */
__STATIC_INLINE void nrf_spis_shorts_enable(NRF_SPIS_Type * p_spis,
                                            uint32_t spis_shorts_mask);

/**
 * @brief Function for disabling specified shortcuts.
 *
 * @param[in] p_spis           SPIS instance.
 * @param[in] spis_shorts_mask Shortcuts to disable.
 */
__STATIC_INLINE void nrf_spis_shorts_disable(NRF_SPIS_Type * p_spis,
                                             uint32_t spis_shorts_mask);

/**
 * @brief Function for enabling specified interrupts.
 *
 * @param[in] p_spis        SPIS instance.
 * @param[in] spis_int_mask Interrupts to enable.
 */
__STATIC_INLINE void nrf_spis_int_enable(NRF_SPIS_Type * p_spis,
                                         uint32_t spis_int_mask);

/**
 * @brief Function for disabling specified interrupts.
 *
 * @param[in] p_spis        SPIS instance.
 * @param[in] spis_int_mask Interrupts to disable.
 */
__STATIC_INLINE void nrf_spis_int_disable(NRF_SPIS_Type * p_spis,
                                          uint32_t spis_int_mask);

/**
 * @brief Function for retrieving the state of a given interrupt.
 *
 * @param[in] p_spis   SPIS instance.
 * @param[in] spis_int Interrupt to check.
 *
 * @retval true  If the interrupt is enabled.
 * @retval false If the interrupt is not enabled.
 */
__STATIC_INLINE bool nrf_spis_int_enable_check(NRF_SPIS_Type const * p_spis,
                                               nrf_spis_int_mask_t spis_int);

/**
 * @brief Function for enabling the SPIS peripheral.
 *
 * @param[in] p_spis SPIS instance.
 */
__STATIC_INLINE void nrf_spis_enable(NRF_SPIS_Type * p_spis);

/**
 * @brief Function for disabling the SPIS peripheral.
 *
 * @param[in] p_spis SPIS instance.
 */
__STATIC_INLINE void nrf_spis_disable(NRF_SPIS_Type * p_spis);

/**
 * @brief Function for retrieving the SPIS semaphore status.
 *
 * @param[in] p_spis SPIS instance.
 *
 * @returns Current semaphore status.
 */
__STATIC_INLINE nrf_spis_semstat_t nrf_spis_semaphore_status_get(NRF_SPIS_Type * p_spis);

/**
 * @brief Function for retrieving the SPIS status.
 *
 * @param[in] p_spis SPIS instance.
 *
 * @returns Current SPIS status.
 */
__STATIC_INLINE nrf_spis_status_mask_t nrf_spis_status_get(NRF_SPIS_Type * p_spis);

/**
 * @brief Function for configuring SPIS pins.
 *
 * If a given signal is not needed, pass the @ref NRF_SPIS_PIN_NOT_CONNECTED
 * value instead of its pin number.
 *
 * @param[in] p_spis    SPIS instance.
 * @param[in] sck_pin   SCK pin number.
 * @param[in] mosi_pin  MOSI pin number.
 * @param[in] miso_pin  MISO pin number.
 * @param[in] csn_pin   CSN pin number.
 */
__STATIC_INLINE void nrf_spis_pins_set(NRF_SPIS_Type * p_spis,
                                       uint32_t sck_pin,
                                       uint32_t mosi_pin,
                                       uint32_t miso_pin,
                                       uint32_t csn_pin);

/**
 * @brief Function for setting the transmit buffer.
 *
 * @param[in]  p_spis   SPIS instance.
 * @param[in]  p_buffer Pointer to the buffer that contains the data to send.
 * @param[in]  length   Maximum number of data bytes to transmit.
 */
__STATIC_INLINE void nrf_spis_tx_buffer_set(NRF_SPIS_Type * p_spis,
                                            uint8_t const * p_buffer,
                                            uint8_t         length);

/**
 * @brief Function for setting the receive buffer.
 *
 * @param[in] p_spis   SPIS instance.
 * @param[in] p_buffer Pointer to the buffer for received data.
 * @param[in] length   Maximum number of data bytes to receive.
 */
__STATIC_INLINE void nrf_spis_rx_buffer_set(NRF_SPIS_Type * p_spis,
                                            uint8_t * p_buffer,
                                            uint8_t   length);

/**
 * @brief Function for getting the number of bytes transmitted
 *        in the last granted transaction.
 *
 * @param[in]  p_spis   SPIS instance.
 *
 * @returns Number of bytes transmitted.
 */
__STATIC_INLINE uint8_t nrf_spis_tx_amount_get(NRF_SPIS_Type const * p_spis);

/**
 * @brief Function for getting the number of bytes received
 *        in the last granted transaction.
 *
 * @param[in]  p_spis   SPIS instance.
 *
 * @returns Number of bytes received.
 */
__STATIC_INLINE uint8_t nrf_spis_rx_amount_get(NRF_SPIS_Type const * p_spis);

/**
 * @brief Function for setting the SPI configuration.
 *
 * @param[in] p_spis        SPIS instance.
 * @param[in] spi_mode      SPI mode.
 * @param[in] spi_bit_order SPI bit order.
 */
__STATIC_INLINE void nrf_spis_configure(NRF_SPIS_Type * p_spis,
                                        nrf_spis_mode_t spi_mode,
                                        nrf_spis_bit_order_t spi_bit_order);

/**
 * @brief Function for setting the default character.
 *
 * @param[in] p_spis SPIS instance.
 * @param[in] def    Default character that is clocked out in case of
 *                   an overflow of the RXD buffer.
 */
__STATIC_INLINE void nrf_spis_def_set(NRF_SPIS_Type * p_spis,
                                      uint8_t def);

/**
 * @brief Function for setting the over-read character.
 *
 * @param[in] p_spis SPIS instance.
 * @param[in] orc    Over-read character that is clocked out in case of
 *                   an over-read of the TXD buffer.
 */
__STATIC_INLINE void nrf_spis_orc_set(NRF_SPIS_Type * p_spis,
                                      uint8_t orc);


#ifndef SUPPRESS_INLINE_IMPLEMENTATION

__STATIC_INLINE void nrf_spis_task_trigger(NRF_SPIS_Type * p_spis,
                                           nrf_spis_task_t spis_task)
{
    *((volatile uint32_t *)((uint8_t *)p_spis + (uint32_t)spis_task)) = 0x1UL;
}

__STATIC_INLINE uint32_t nrf_spis_task_address_get(NRF_SPIS_Type const * p_spis,
                                                   nrf_spis_task_t spis_task)
{
    return (uint32_t)p_spis + (uint32_t)spis_task;
}   

__STATIC_INLINE void nrf_spis_event_clear(NRF_SPIS_Type *  p_spis,
                                          nrf_spis_event_t spis_event)
{
    *((volatile uint32_t *)((uint8_t *)p_spis + (uint32_t)spis_event)) = 0x0UL;
}

__STATIC_INLINE bool nrf_spis_event_check(NRF_SPIS_Type const * p_spis,
                                          nrf_spis_event_t spis_event)
{
    return (bool)*(volatile uint32_t *)((uint8_t *)p_spis + (uint32_t)spis_event);
}

__STATIC_INLINE uint32_t nrf_spis_event_address_get(NRF_SPIS_Type const * p_spis,
                                                    nrf_spis_event_t spis_event)
{
    return (uint32_t)p_spis + (uint32_t)spis_event;
}

__STATIC_INLINE void nrf_spis_shorts_enable(NRF_SPIS_Type * p_spis,
                                            uint32_t spis_shorts_mask)
{
    p_spis->SHORTS |= spis_shorts_mask;
}

__STATIC_INLINE void nrf_spis_shorts_disable(NRF_SPIS_Type * p_spis,
                                             uint32_t spis_shorts_mask)
{
    p_spis->SHORTS &= ~(spis_shorts_mask);
}

__STATIC_INLINE void nrf_spis_int_enable(NRF_SPIS_Type * p_spis,
                                         uint32_t spis_int_mask)
{
    p_spis->INTENSET = spis_int_mask;
}

__STATIC_INLINE void nrf_spis_int_disable(NRF_SPIS_Type * p_spis,
                                          uint32_t spis_int_mask)
{
    p_spis->INTENCLR = spis_int_mask;
}

__STATIC_INLINE bool nrf_spis_int_enable_check(NRF_SPIS_Type const * p_spis,
                                               nrf_spis_int_mask_t spis_int)
{
    return (bool)(p_spis->INTENSET & spis_int);
}

__STATIC_INLINE void nrf_spis_enable(NRF_SPIS_Type * p_spis)
{
    p_spis->ENABLE = (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);
}

__STATIC_INLINE void nrf_spis_disable(NRF_SPIS_Type * p_spis)
{
    p_spis->ENABLE = (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos);
}

__STATIC_INLINE nrf_spis_semstat_t nrf_spis_semaphore_status_get(NRF_SPIS_Type * p_spis)
{
    return (nrf_spis_semstat_t) ((p_spis->SEMSTAT & SPIS_SEMSTAT_SEMSTAT_Msk) 
                                 >> SPIS_SEMSTAT_SEMSTAT_Pos);
}

__STATIC_INLINE nrf_spis_status_mask_t nrf_spis_status_get(NRF_SPIS_Type * p_spis)
{
    return (nrf_spis_status_mask_t) p_spis->STATUS;
}

__STATIC_INLINE void nrf_spis_pins_set(NRF_SPIS_Type * p_spis,
                                       uint32_t sck_pin,
                                       uint32_t mosi_pin,
                                       uint32_t miso_pin,
                                       uint32_t csn_pin)
{
    p_spis->PSELSCK  = sck_pin;
    p_spis->PSELMOSI = mosi_pin;
    p_spis->PSELMISO = miso_pin;
    p_spis->PSELCSN  = csn_pin;
}

__STATIC_INLINE void nrf_spis_tx_buffer_set(NRF_SPIS_Type * p_spis,
                                            uint8_t const * p_buffer,
                                            uint8_t         length)
{
    p_spis->TXDPTR = (uint32_t)p_buffer;
    p_spis->MAXTX  = length;
}

__STATIC_INLINE void nrf_spis_rx_buffer_set(NRF_SPIS_Type * p_spis,
                                            uint8_t * p_buffer,
                                            uint8_t   length)
{
    p_spis->RXDPTR = (uint32_t)p_buffer;
    p_spis->MAXRX  = length;
}

__STATIC_INLINE uint8_t nrf_spis_tx_amount_get(NRF_SPIS_Type const * p_spis)
{
    return (uint8_t) p_spis->AMOUNTRX;
}

__STATIC_INLINE uint8_t nrf_spis_rx_amount_get(NRF_SPIS_Type const * p_spis)
{
    return (uint8_t) p_spis->AMOUNTTX;
}

__STATIC_INLINE void nrf_spis_configure(NRF_SPIS_Type * p_spis,
                                        nrf_spis_mode_t spi_mode,
                                        nrf_spis_bit_order_t spi_bit_order)
{
    uint32_t config = (spi_bit_order == NRF_SPIS_BIT_ORDER_MSB_FIRST ?
        SPIS_CONFIG_ORDER_MsbFirst : SPIS_CONFIG_ORDER_LsbFirst);

    switch (spi_mode)
    {
    default:
    case NRF_SPIS_MODE_0:
        config |= (SPIS_CONFIG_CPOL_ActiveHigh << SPIS_CONFIG_CPOL_Pos) |
                  (SPIS_CONFIG_CPHA_Leading    << SPIS_CONFIG_CPHA_Pos);
        break;

    case NRF_SPIS_MODE_1:
        config |= (SPIS_CONFIG_CPOL_ActiveHigh << SPIS_CONFIG_CPOL_Pos) |
                  (SPIS_CONFIG_CPHA_Trailing   << SPIS_CONFIG_CPHA_Pos);
        break;

    case NRF_SPIS_MODE_2:
        config |= (SPIS_CONFIG_CPOL_ActiveLow  << SPIS_CONFIG_CPOL_Pos) |
                  (SPIS_CONFIG_CPHA_Leading    << SPIS_CONFIG_CPHA_Pos);
        break;

    case NRF_SPIS_MODE_3:
        config |= (SPIS_CONFIG_CPOL_ActiveLow  << SPIS_CONFIG_CPOL_Pos) |
                  (SPIS_CONFIG_CPHA_Trailing   << SPIS_CONFIG_CPHA_Pos);
        break;
    }
    p_spis->CONFIG = config;
}

__STATIC_INLINE void nrf_spis_orc_set(NRF_SPIS_Type * p_spis,
                                      uint8_t orc)
{
    p_spis->ORC = orc;
}

__STATIC_INLINE void nrf_spis_def_set(NRF_SPIS_Type * p_spis,
                                      uint8_t def)
{
    p_spis->DEF = def;
}

#endif // SUPPRESS_INLINE_IMPLEMENTATION

#ifdef __cplusplus
}
#endif

#endif // NRF_SPIS_H__

/** @} */
