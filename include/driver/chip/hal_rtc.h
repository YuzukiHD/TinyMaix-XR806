/**
 * @file  hal_rtc.h
 * @author  XRADIO IOT WLAN Team
 */

/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _DRIVER_CHIP_HAL_RTC_H_
#define _DRIVER_CHIP_HAL_RTC_H_

#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RTC register block structure
 */
typedef struct {
	__IO uint32_t CTRL;                 /* offset: 0x00, RTC control register */
	     uint32_t RESERVED0[3];
	__IO uint32_t YYMMDD;               /* offset: 0x10, RTC YYMMDD Register */
	__IO uint32_t DDHHMMSS;             /* offset: 0x14, RTC HHMMSS register */
	     uint32_t RESERVED1[2];
	__IO uint32_t SEC_ALARM_LOAD_VAL;   /* offset: 0x20, RTC second alarm load/interval value register */
	__IO uint32_t SEC_ALARM_CUR_VAL;    /* offset: 0x24, RTC second alarm current value register */
	__IO uint32_t SEC_ALARM_EN;         /* offset: 0x28, RTC second alarm enable register */
	__IO uint32_t SEC_ALARM_IRQ_EN;     /* offset: 0x2C, RTC second alarm IRQ enable register */
	__IO uint32_t SEC_ALARM_IRQ_STATUS; /* offset: 0x30, RTC second alarm IRQ status register */
	     uint32_t RESERVED2[3];
	__IO uint32_t WDAY_ALARM_HHMMSS;    /* offset: 0x40, RTC week day alarm HHMMSS register */
	__IO uint32_t WDAY_ALARM_WDAY_EN;   /* offset: 0x44, RTC week day alarm enable register */
	__IO uint32_t WDAY_ALARM_IRQ_EN;    /* offset: 0x48, RTC week day alarm IRQ enable register */
	__IO uint32_t WDAY_ALARM_IRQ_STATUS;/* offset: 0x4C, RTC week day alarm IRQ status register */
	__IO uint32_t ALARM_WAKEUP_EN;      /* offset: 0x50, RTC second and week day alarm wakeup register */
	     uint32_t RESERVED3[3];
	__IO uint32_t FREERUN_CNT_L;        /* offset: 0x60, Free running counter low register */
	__IO uint32_t FREERUN_CNT_H;        /* offset: 0x64, Free running counter high register */
} RTC_T;

#define RTC ((RTC_T *)RTC_BASE)         /* address: 0x40041800*/

/*
 * Bit field definition of RTC->CTRL
 */
#define RTC_TEST_MODE_BIT                   HAL_BIT(31)
#define RTC_SIMULATION_BIT                  HAL_BIT(30)
#define RTC_WDAY_ALARM_HHMMSS_ACCESS_BIT    HAL_BIT(2)
#define RTC_DDHHMMSS_ACCESS_BIT             HAL_BIT(1)
#define RTC_YYMMDD_ACCESS_BIT               HAL_BIT(0)

/*
 * Bit field definition of RTC->YYMMDD
 */
#define RTC_LEAP_YEAR_BIT   HAL_BIT(24)

#define RTC_YEAR_SHIFT      16  /* R/W, [0, 255] */
#define RTC_YEAR_VMASK      0xFF
#define RTC_YEAR_MIN        0
#define RTC_YEAR_MAX        255

#define RTC_MONTH_SHIFT     8   /* R/W, [1, 12] */
#define RTC_MONTH_VMASK     0xF
#define RTC_MONTH_MIN       1
#define RTC_MONTH_MAX       12

#define RTC_MDAY_SHIFT      0   /* R/W, [1, 31] */
#define RTC_MDAY_VMASK      0x1F
#define RTC_MDAY_MIN        1
#define RTC_MDAY_MAX        31

/*
 * Bit field definition of RTC->DDHHMMSS
 */
#define RTC_WDAY_SHIFT      29  /* R/W */
#define RTC_WDAY_VMASK      0x7
typedef enum {
	RTC_WDAY_MONDAY         = 0U,
	RTC_WDAY_TUESDAY        = 1U,
	RTC_WDAY_WEDNESDAY      = 2U,
	RTC_WDAY_THURSDAY       = 3U,
	RTC_WDAY_FRIDAY         = 4U,
	RTC_WDAY_SATURDAY       = 5U,
	RTC_WDAY_SUNDAY         = 6U
} RTC_WeekDay;

#define RTC_HOUR_SHIFT      16  /* R/W, [0, 23] */
#define RTC_HOUR_VMASK      0x1F
#define RTC_HOUR_MIN        0
#define RTC_HOUR_MAX        23

#define RTC_MINUTE_SHIFT    8   /* R/W, [0, 59] */
#define RTC_MINUTE_VMASK    0x3F
#define RTC_MINUTE_MIN      0
#define RTC_MINUTE_MAX      59

#define RTC_SECOND_SHIFT    0   /* R/W, [0, 59] */
#define RTC_SECOND_VMASK    0x3F
#define RTC_SECOND_MIN      0
#define RTC_SECOND_MAX      59

/* RTC->SEC_ALARM_LOAD_VAL */

/* RTC->SEC_ALARM_CUR_VAL */

/* RTC->SEC_ALARM_EN */
#define RTC_SEC_ALARM_EN_BIT            HAL_BIT(0)

/* RTC->SEC_ALARM_IRQ_EN */
#define RTC_SEC_ALARM_IRQ_EN_BIT        HAL_BIT(0)

/* RTC->SEC_ALARM_IRQ_STATUS */
#define RTC_SEC_ALARM_IRQ_PENDING_BIT   HAL_BIT(0)

/* RTC->WDAY_ALARM_DDHHMMSS */
#define RTC_WDAY_ALARM_HOUR_SHIFT       16  /* R/W, [0, 23] */
#define RTC_WDAY_ALARM_HOUR_VMASK       0x1F

#define RTC_WDAY_ALARM_MINUTE_SHIFT     8   /* R/W, [0, 59] */
#define RTC_WDAY_ALARM_MINUTE_VMASK     0x3F

#define RTC_WDAY_ALARM_SECOND_SHIFT     0   /* R/W, [0, 59] */
#define RTC_WDAY_ALARM_SECOND_VMASK     0x3F

/* RTC->WDAY_ALARM_EN */
#define RTC_WDAY_ALARM_EN_BIT(wday)     HAL_BIT(wday)   /* day is RTC_WeekDay */
#define RTC_WDAY_ALARM_EN_MASK          0x7F

/* RTC->WDAY_ALARM_IRQ_EN */
#define RTC_WDAY_ALARM_IRQ_EN_BIT       HAL_BIT(0)

/* RTC->WDAY_ALARM_IRQ_STATUS */
#define RTC_WDAY_ALARM_IRQ_PENDING_BIT  HAL_BIT(0)

/* RTC->ALARM_WAKEUP_EN */
#define RTC_SEC_ALARM_WAKEUP_EN_BIT     HAL_BIT(0)
#define RTC_WDAY_ALARM_WAKEUP_EN_BIT    HAL_BIT(1)

/******************************************************************************/

/** @brief Type define of RTC alarm IRQ callback function */
typedef void (*RTC_AlarmIRQCallback) (void *arg);

/**
 * @brief RTC second alarm starting parameters
 */
typedef struct {
	uint32_t                alarmSeconds;   /* RTC second alarm's count down value */
	RTC_AlarmIRQCallback    callback;       /* RTC second alarm IRQ callback function */
	void                    *arg;           /* Argument of RTC second alarm IRQ callback function */
} RTC_SecAlarmStartParam;

/**
 * @brief RTC weekday alarm starting parameters
 */
typedef struct {
	uint8_t                 alarmHour;      /* RTC weekday alarm's hour, [0, 23] */
	uint8_t                 alarmMinute;    /* RTC weekday alarm's minute, [0, 59] */
	uint8_t                 alarmSecond;    /* RTC weekday alarm's second, [0, 59] */
	uint8_t                 alarmWDayMask;  /* RTC weekday alarm's weekday, bit mask of RTC_WDAY_ALARM_EN_BIT(RTC_WeekDay) */
	RTC_AlarmIRQCallback    callback;       /* RTC weekday alarm IRQ callback function */
	void                    *arg;           /* Argument of RTC weekday alarm IRQ callback function */
} RTC_WDayAlarmStartParam;

/**
 * @brief Set the RTC date, including leaf year flag, year, month and month day
 * @param[in] isLeapYear Leap year flag set to the RTC.
 *     @arg !0 The year is a leap year
 *     @arg  0 The year is not a leap year
 * @param[in] year Year set to the RTC, [0, 255]
 * @param[in] month Month set to the RTC, [1, 12]
 * @param[in] mday Month day set to the RTC, [1, 31]
 * @return None
 *
 * @note The leap year flag is always set by the caller, but never changed after
 *       setting. So the leap year bit of the RTC maybe wrong.
 * @note The value of year is not a real year, but year's offset relative to
 *       the base year defined by the caller.
 * @note The correction of the combination of all the parameters is guaranteed
 *       by the caller.
 */
void HAL_RTC_SetYYMMDD(uint8_t isLeapYear, uint8_t year, uint8_t month, uint8_t mday);

/**
 * @brief Set the RTC weekday and time including hour, minute and second
 * @param[in] wday Weekday set to the RTC
 * @param[in] hour Hour set to the RTC, [0, 23]
 * @param[in] minute Minute set to the RTC, [0, 59]
 * @param[in] second Second set to the RTC, [0, 59]
 * @return None
 *
 * @note The correction of the weekday is guaranteed by the caller.
 */
void HAL_RTC_SetDDHHMMSS(RTC_WeekDay wday, uint8_t hour, uint8_t minute, uint8_t second);

void HAL_RTC_SetLeapYear(uint8_t isLeapYear);

/**
 * @brief Get the RTC date, including leaf year flag, year, month and month day
 * @param[out] isLeapYear The RTC's leap year flag. Don't use it because it
 *                        maybe wrong.
 *     - 1 means the year is a leap year
 *     - 0 means the year is not a leap year
 * @param[out] year The RTC's Year
 * @param[out] month The RTC's Month
 * @param[out] mday The RTC's Month day
 * @return None
 *
 * @note Don't trust the RTC leap year flag, because it's never changed by the
 *       RTC hardware, and maybe wrong.
 * @note The RTC's year is not a real year, but year offset relative to the
 *       base year defined by the caller.
 */
void HAL_RTC_GetYYMMDD(uint8_t *isLeapYear, uint8_t *year, uint8_t *month, uint8_t *mday);

/**
 * @brief Get the RTC weekday and time including hour, minute and second
 * @param[out] wday The RTC's Weekday
 * @param[out] hour The RTC's hour
 * @param[out] minute The RTC's minute
 * @param[out] second The RTC's second
 * @return None
 */
void HAL_RTC_GetDDHHMMSS(RTC_WeekDay *wday, uint8_t *hour, uint8_t *minute, uint8_t *second) ;

/**
 * @brief Start the RTC second alarm once
 *
 * After starting, the RTC second alarm counts down from param->alarmSeconds
 * to zero, and trigger interrupt when it reach zero. After alarming, the
 * alarm stops automatically.
 *
 * @param[in] param Pointer to RTC_SecAlarmStartParam structure
 * @return None
 *
 */
void HAL_RTC_StartSecAlarm(const RTC_SecAlarmStartParam *param);

/**
 * @brief Stop the RTC second alarm
 * @return None
 */
void HAL_RTC_StopSecAlarm(void);

/**
 * @brief Start the RTC weekday alarm
 *
 * After starting, the RTC weekday alarm will trigger interrupt when it reach
 * the configured weekday time. After alarming, the alarm continues to running.
 *
 * @param[in] param Pointer to RTC_WDayAlarmStartParam structure
 * @return None
 */
void HAL_RTC_StartWDayAlarm(const RTC_WDayAlarmStartParam *param);

/**
 * @brief Stop the RTC weekday alarm
 * @return None
 */
void HAL_RTC_StopWDayAlarm(void);

/**
 * @brief Init the RTC module
 * @return None
 */
void HAL_RTC_Init(void);

/**
 * @brief Convert RTC's free running counts to time value (in microsecond)
 * @param[in] cnt RTC's free running counts to be converted
 * @return Time value (in microsecond) after converted
 */
uint64_t HAL_RTC_FreeRunCntToTime(uint64_t cnt);

/**
 * @brief Convert time value (in microsecond) to RTC's Free running counts
 * @param[in] us Time value (in microsecond) to be converted
 * @return RTC's Free running counts after converted
 */
uint64_t HAL_RTC_FreeRunTimeToCnt(uint64_t us);

/**
 * @brief Get The counts of the RTC's Free running counter
 *
 * Free running counter is a 48-bit counter which is driven by LFCLK and starts
 * to count as soon as the system reset is released and the LFCLK is ready.
 *
 * @return The counts of RTC's Free running counter
 */
uint64_t HAL_RTC_GetFreeRunCnt(void);

/**
 * @brief Get the time value (in microsecond) of the RTC's Free running counter
 *
 * Free running counter is a 48-bit counter which is driven by LFCLK and starts
 * to count as soon as the system reset is released and the LFCLK is ready.
 *
 * The time unit of the counter is: (10^6 / LFCLK) us
 *
 * @return The time value (in microsecond) of the RTC's Free running counter.
 *         Its accuracy is about 32 us.
 */
uint64_t HAL_RTC_GetFreeRunTime(void);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_RTC_H_ */
