/******************************************************************************
Copyright (c) 2019 Denis Artyomov (denis.artyomov@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#define F_CPU 8000000

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
/* Основные настройки */

// Максимальное выходное напряжение ручки газа в милливольтах
#ifndef MaxVg
	#define MaxVg 3600
#endif

// Максимальное напряжение на входе контроллера, после которого
// колесо перестаёт набирать обороты
#ifndef MaxVk
	#define MaxVk 3600
#endif

// Минимальное напряжение на входе контроллера, на котором начинает крутиться колесо
#ifndef MinV
	#define MinV 1200
#endif

// Степень нелинейности (от 0 до 5).
// 0 - линейная характеристика
// 3 - нелинейная
// 5 - сильно нелинейная
#ifndef K
	#define K 3
#endif

// Максимальное время набора скорости в секундах
// Выходной сигнал от 0 до MaxVk будет нарастать за это время
#ifndef MaxGainTime
	#define MaxGainTime 3
#endif

// Максимальное время спада скорости в секундах
// Выходной сигнал от MaxVk до 0 будет спадать за это время
#ifndef MaxDropTime
	#define MaxDropTime 1
#endif

// Одна запись таблицы трансляции
struct AdcPwmItem
{
	uint16_t in_mv;  // входное напряжение в милливольтах
	uint16_t out_mv; // выходное напряжение в милливольтах
};

// Точки перегиба на кривой. Их вручную менять не надо
constexpr uint16_t MidIn1  = (15L-2L + K/2) * (MinV + MaxVg) / 30L;
constexpr uint16_t MidOut1 = (15L-2L - K) * (MinV + MaxVk) / 30L;
constexpr uint16_t MidIn2  = (15L+2L + K/2) * (MinV + MaxVg) / 30L;
constexpr uint16_t MidOut2 = (15L+2L - K) * (MinV + MaxVk) / 30L;

// Таблица для основного ведущего колеса
// Если у вас одноприводный девайс, то надо менять именно эту таблицу
// Значения в виде {Vin, Vout} в милливольтах
static const AdcPwmItem transl_table1[] = {
	{0,      0      },
	{MinV,   MinV   },
	{MidIn1, MidOut1},
	{MidIn2, MidOut2},
	{MaxVg,  MaxVk  },
	{5500,   5500   }
};

// Таблица для колеса с редуктором
static const AdcPwmItem transl_table2[] = {
	{0,      0      },
	{MinV,   MinV   },
	{MidIn1, MidOut1},
	{MidIn2, MidOut2},
	{MaxVg,  MidOut2},
	{5500,   5500   }
};

// Коэфициент усиления на выходе в процентах
// Нужен чтобы компенсировать низкое входное сопротивление входа
// контроллера для ручки газа, из-за которого просаживается
// выходное напряжение с RC-цепочки
#ifndef OutGain
	#define OutGain 100
#endif

///////////////////////////////////////////////////////////////////////////////

// Частота обработки данных
constexpr uint32_t WorkFreq = 20;

// Выходной пин ШИМ для основного колеса
#define Pwm1Pin PB1

// Выходной пин ШИМ для редукторного колеса
#define Pwm2Pin PB4

// Канал АЦП для чтения значения с курка
constexpr uint8_t AdcChan = 3;

// Максимальное значение с АЦП
constexpr uint32_t MaxAdc = 1023;

// Максимально возможное значение ШИМ
constexpr uint32_t MaxPwm = 1023;

// Делитель частоты таймера для отсчёта периода
constexpr uint32_t PeriodTimerPrescaler = 1024;

// Т.к. не хватает делителя таймера отсчёта периода, есть ешё
// счётчик тиков таймера
constexpr uint8_t PeriodTimerCnt = 4;

///////////////////////////////////////////////////////////////////////////////

// Инициализация АЦП
static void init_adc()
{
	ADCSRA =
		_BV(ADEN) |             // Enable ADC
		_BV(ADPS2) | _BV(ADPS0) // clk div 32
	;

	ADMUX = 0; // ref. voltage = Vcc
}

// Инициализация таймера 1 для выдачи ШИМ на пины
static void init_pwm()
{
	TCCR1 =
		_BV(PWM1A)  | // Pulse Width Modulator A Enable
		_BV(COM1A0) | // Comparator A Output Mode = PWM
		_BV(CS11);    // clk/2

	GTCCR =
		_BV(PWM1B) | // Pulse Width Modulator B Enable
		_BV(COM1B0); // Comparator B Output Mode = PWM


	OCR1A = 0;
	OCR1B = 0;

	DDRB =
		_BV(Pwm1Pin) | // Пины ШИМ на выход
		_BV(Pwm2Pin);

	TIMSK = _BV(TOIE1);
}

// Инициализация таймера для отсчёта периода замера
static void init_period_timer()
{
	TCCR0A =
		_BV(WGM01); // CTC mode

	TCCR0B =
		_BV(CS00)|_BV(CS02); // clk/1024

	// Делаем чтобы таймер тикал с частотой WorkFreq*PeriodTimerCnt
	constexpr uint32_t TimerPeriod = (F_CPU / (WorkFreq * PeriodTimerPrescaler * (uint32_t)PeriodTimerCnt));
	static_assert((TimerPeriod > 1) && (TimerPeriod <= 255), "Wrong WorkFreq or F_CPU");
	OCR0A = TimerPeriod;
}

///////////////////////////////////////////////////////////////////////////////

// Глобальные переменные для эмуляции 10-разрядного ШИМ

static volatile uint8_t pwm_values1[4] = {0};
static volatile uint8_t pwm_values2[4] = {0};
static volatile uint8_t pwm_cycle = 0;

// Прерывание по переполнению таймера для эмуляции 10-разрядного ШИМ
ISR (TIMER1_OVF_vect)
{
	OCR1A = pwm_values1[pwm_cycle];
	OCR1B = pwm_values2[pwm_cycle];
	pwm_cycle = (pwm_cycle + 1) & 0x03;
}

// Установка значений ШИМ для 2-х каналов
static void set_pwm_values(uint16_t value1, uint16_t value2)
{
	auto set_pwm_vector = [] (uint16_t value, volatile uint8_t *vector)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			if (value >= 256)
			{
				*vector++ = 255;
				value -= 256;
			}
			else
			{
				*vector++ = value;
				value = 0;
			}
		}
	};

	cli();
	set_pwm_vector(value1, pwm_values1);
	set_pwm_vector(value2, pwm_values2);
	sei();
}

// Чтение значения АЦП по указанному каналу
static uint16_t read_adc_value(uint8_t channel, bool delay_2_ms)
{
	// Выбираем канал АЦП
	ADMUX = (ADMUX & ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2) | _BV(MUX3))) | channel;

	// Делаем задержку по необходимости
	// а она нужна перед чтением эталонного источника 1.1в
	if (delay_2_ms) _delay_ms(2);

	// Запускаем преобразование
	ADCSRA |= _BV(ADSC);

	// Ждём окончания преобразования
	while (ADCSRA & _BV(ADSC)) {}

	// Читаем и возвращаем значение АЦП
	return ADC;
}

// Получение среднего значения после того, как из набора
// выброшено 1/3 максимальных и 1/3 минимальных значений
static uint16_t get_median_aver_value(int16_t *values, const uint8_t size)
{
	// Выбрасываем 1/3 и 1/3 минимальных и максимальных значений,
	// заменяя их на -1
	for (uint8_t i = 0; i < size/3; i++)
	{
		int8_t max_index = -1;
		int8_t min_index = -1;
		int16_t max = -1;
		int16_t min = -1;
		for (uint8_t i = 0; i < size; i++)
		{
			auto val = values[i];
			if (val == -1) continue;
			if ((max == -1) || (val > max))
			{
				max_index = i;
				max = val;
			}
			if ((min == -1) || (val < min))
			{
				min_index = i;
				min = val;
			}
		}
		if (min_index != -1)
			values[min_index] = -1;

		if (max_index != -1)
			values[max_index] = -1;
	}

	// Для оставшихся считаем среднее значение
	uint32_t acc = 0;
	uint32_t cnt = 0;
	for (uint8_t i = 0; i < size; i++)
	{
		auto val = values[i];
		if (val == -1) continue;
		acc += val;
		cnt++;
	}

	return (cnt != 0) ? ((acc + cnt/2) / cnt) : 0;
}

// Чтение значения по указанному каналу несколько раз и выдача фильтрованного значения
static uint16_t read_filtered_adc_value(uint8_t channel, bool delay_2_ms)
{
	constexpr uint8_t size = 10;
	int16_t values[size];

	for (auto &adc_value : values)
		adc_value = read_adc_value(channel, delay_2_ms);

	return get_median_aver_value(values, size);
}

// Вычисление линейной интерполяции
static int32_t line_interpolate(int32_t x, int32_t x1, int32_t x2, int32_t y1, int32_t y2)
{
	int32_t tmp = x2 - x1;
	return ((x - x1) * (y2 - y1) + tmp/2) / tmp + y1;
}

// Перевод значения напряжения из входного в выходное по таблице.
// Значения напряжения - в милливольтах
static uint16_t translate_volatge(uint16_t value, const AdcPwmItem *table, const uint8_t table_size)
{
	for (uint8_t i = 0; i < (table_size-1); i++)
	{
		auto item1 = table[i];
		auto item2 = table[i+1];

		if ((item1.in_mv <= value) && (value <= item2.in_mv))
			return line_interpolate(value, item1.in_mv, item2.in_mv, item1.out_mv, item2.out_mv);
	}
	return 0;
}

// Перевод значения АЦП в напряжение
static uint16_t adc_to_voltage(uint16_t adc_value, uint16_t rev_voltage)
{
	return (uint32_t)adc_value * (uint32_t)rev_voltage / (uint32_t)MaxAdc;
}

// Перевод значения напряжения в величину ШИМ для таймера
static uint16_t voltage_to_pwm(uint16_t voltage, uint16_t rev_voltage)
{
	if (rev_voltage == 0) return 0;
	uint32_t value = MaxPwm * (uint32_t)voltage / (uint32_t)rev_voltage;
	if (value > MaxPwm) value = MaxPwm;
	return value;
}

// Обработка текущей ситуации для одного колеса
static uint16_t process_for_channel(
	uint16_t         in_voltage,
	uint16_t         adc_ref_voltage,
	int16_t          &smooth_voltage,
	const AdcPwmItem *table,
	const uint8_t    table_size)
{
	// Преобразуем входное напряжение в выходное по таблице
	auto out_voltage = translate_volatge(in_voltage, table, table_size);

	// Если напряжение меньше MinV, то просто выдаём его на выход
	if (out_voltage < MinV)
	{
		smooth_voltage = out_voltage;
	}

	// ... иначе плавно приближаем smooth_voltage к out_voltage
	else
	{
		int16_t diff = out_voltage - smooth_voltage;
		constexpr int16_t MaxGainDiff = (int32_t)MaxVk / ((int32_t)MaxGainTime * WorkFreq);
		if (diff > MaxGainDiff) diff = MaxGainDiff;
		constexpr int16_t MaxDropDiff = (int32_t)MaxVk / ((int32_t)MaxDropTime * WorkFreq);
		if (diff < -MaxDropDiff) diff = -MaxDropDiff;
		smooth_voltage += diff;
	}

	// Возвращаем значение для ШИМ
	return voltage_to_pwm(((uint32_t)OutGain * (uint32_t)smooth_voltage + 50UL) / 100UL, adc_ref_voltage);
}

// Ожидание когда тикнет таймер, который отсчитывает период
static void wait_for_period_timer()
{
	for (uint8_t i = 0; i < PeriodTimerCnt; i++)
	{
		while (!(TIFR & OCF0A)) {}
		TIFR = OCF0A;
	}
}

int main()
{
	// "Плавные" значения выходного напряжения
	int16_t smooth_voltage1 = 0;
	int16_t smooth_voltage2 = 0;

	// Инициализируем АЦП
	init_adc();

	// Инициализируем ШИМ и пины
	init_pwm();

	// Ждём 10 мс на всякий случай
	_delay_ms(10);

	// Инициализируем таймер, который отсчитывает период для замера
	init_period_timer();

	for (;;)
	{
		// Ждём пока не тикнет таймер, который отсчитывает период
		wait_for_period_timer();

		// Читаем значение эталонного источника 1.1 вольт
		uint16_t adc1100 = read_filtered_adc_value(0b1100, true);
		if (adc1100 == 0) continue; // "не заводимся", если что-то пошло не так

		// Получаем текущее значение напряжения питания
		uint16_t adc_ref_voltage = (uint32_t)MaxAdc * 1100UL / (uint32_t)adc1100;

		// Читаем значение АЦП с курка
		uint16_t adc_value = read_filtered_adc_value(AdcChan, false);

		// Переводим значение с курка в милливольты
		uint16_t in_voltage = adc_to_voltage(adc_value, adc_ref_voltage);

		// Производим обработку для основного колеса
		uint16_t pwm_value1 = process_for_channel(
			in_voltage,
			adc_ref_voltage,
			smooth_voltage1,
			transl_table1,
			sizeof(transl_table1)/sizeof(AdcPwmItem)
		);

		// Производим обработку для колеса с редуктором
		uint16_t pwm_value2 = process_for_channel(
			in_voltage,
			adc_ref_voltage,
			smooth_voltage2,
			transl_table2,
			sizeof(transl_table2)/sizeof(AdcPwmItem)
		);

		// Выдаём напряжения на выход через ШИМ
		set_pwm_values(pwm_value1, pwm_value2);
	}
}
