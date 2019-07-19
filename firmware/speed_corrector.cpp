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

#define F_CPU 1000000

#include <util/delay.h>
#include <avr/io.h>
#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
/* Основные настройки */

// Максимальное выходное напряжение ручки газа в милливольтах
#define MaxV 4300

// Минимальное напряжение на входе контроллера, на котором начинает крутиться колесо
#define MinV 1200

// Максимальное время набора скорости в секундах
// Выходной сигнал от 0 до MaxV будет нарастать за это время
#define MaxGainTime 3

// Максимальное время спада скорости в секундах
// Выходной сигнал от MaxV до 0 будет спадать за это время
#define MaxDropTime 1

// Одна запись таблицы трансляции
struct AdcPwmItem
{
	uint16_t in_mv;  // входное напряжение в милливольтах
	uint16_t out_mv; // выходное напряжение в милливольтах
};

// Таблица для основного ведущего колеса
// Если у вас одноприводный девайс, то надо менять именно эту таблицу
// Значения в виде {Vin, Vout} в милливольтах
static const AdcPwmItem transl_table1[]  = {
	{   0,    0},
	{MinV, MinV},
	{3000, 1700},
	{3600, 2200},
	{MaxV, MaxV},
	{5500, 5500}
};

// Таблица для колеса с редуктором
static const AdcPwmItem transl_table2[]  = {
	{   0,    0},
	{MinV, MinV},
	{3000, 1700},
	{3600, 2200},
	{4000, 3000},
	{MaxV,    0},
	{5500,    0}
};

///////////////////////////////////////////////////////////////////////////////

// Частота обработки данных
#define WorkFreq 20

// Выходной пин ШИМ для основного колеса
#define Pwm1Pin PB1

// Выходной пин ШИМ для редукторного колеса
#define Pwm2Pin PB4

// Канал АЦП для чтения значения с курка
#define AdcChan 3

// Максимальное значение с АЦП
#define MaxAdc 1023L

// Максимально возможное значение ШИМ
#define MaxPwm 255L

// Делитель частоты таймера доя отсчёта периода
#define PeriodTimerPrescaler 1024

///////////////////////////////////////////////////////////////////////////////

// Инициализация АЦП
static void init_adc()
{
	ADCSRA =
		_BV(ADEN) |             // Enable ADC
		_BV(ADPS1) | _BV(ADPS0) // clk div 8
	;

	ADMUX = 0; // ref. voltage = Vcc
}

// Инициализация таймера 1 для выдачи ШИМ на пины
static void init_pwm()
{
	TCCR1 =
		_BV(PWM1A)  | // Pulse Width Modulator A Enable
		_BV(COM1A0) | // Comparator A Output Mode = PWM
		_BV(CS10);    // clk/1

	GTCCR =
		_BV(PWM1B) | // Pulse Width Modulator B Enable
		_BV(COM1B0); // Comparator B Output Mode = PWM


	OCR1A = 0;
	OCR1B = 0;

	DDRB =
		_BV(Pwm1Pin) | // Пины ШИМ на выход
		_BV(Pwm2Pin);
}

// Инициализация таймера для отсчёта периода замера
static void init_period_timer()
{
	TCCR0A =
		_BV(WGM01); // CTC mode

	TCCR0B =
		_BV(CS00)|_BV(CS02); // clk/1024

	// Делаем чтобы таймер тикал с частотой WorkFreq
	constexpr uint32_t TimerPeriod = (F_CPU / (WorkFreq * PeriodTimerPrescaler));
	static_assert((TimerPeriod > 1) || (TimerPeriod <= 255), "Wrong WorkFreq or F_CPU");
	OCR0A = TimerPeriod;
}

///////////////////////////////////////////////////////////////////////////////

// Чтение значения АЦП по указанному каналу
static uint16_t read_adc_value(uint8_t channel, bool delay_2_ms)
{
	// Выбираем канал АЦП
	ADMUX = (ADMUX & ~(_BV(MUX0) | _BV(MUX1) | _BV(MUX2) | _BV(MUX3))) | channel;

	// Делаем задержку по необходимости
	// а она нужна перед чтением эталонного источника 1.1в
	if (delay_2_ms) _delay_ms(2);

	// Запкскаем преобразование
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
static uint8_t voltage_to_pwm(uint16_t voltage, uint16_t rev_voltage)
{
	if (rev_voltage == 0) return 0;
	int32_t value = (int32_t)MaxPwm * (int32_t)voltage / (int32_t)rev_voltage;
	if (value < 0) value = 0;
	if (value > MaxPwm) value = MaxPwm;
	return value;
}

// Обработка текущей ситуации для одного колеса
static void process_for_channel(
	uint16_t         in_voltage,
	uint16_t         adc_ref_voltage,
	int16_t          &smooth_voltage,
	const AdcPwmItem *table,
	const uint8_t    table_size,
	volatile uint8_t &pwm_register)
{
	// Преобразуем входное напряжение в выходное по таблице
	auto out_voltage = translate_volatge(in_voltage, table, table_size);

	// Плавно приближаем smooth_voltage к out_voltage
	int16_t diff = out_voltage - smooth_voltage;
	constexpr int16_t MaxGainDiff = (int32_t)MaxV / ((int32_t)MaxGainTime * (int32_t)WorkFreq);
	if (diff > MaxGainDiff) diff = MaxGainDiff;
	constexpr int16_t MaxDropDiff = (int32_t)MaxV / ((int32_t)MaxDropTime * (int32_t)WorkFreq);
	if (diff < -MaxDropDiff) diff = -MaxDropDiff;
	smooth_voltage += diff;

	// Записываем значение в регистр ШИМ
	pwm_register = voltage_to_pwm(smooth_voltage, adc_ref_voltage);
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
		while (!(TIFR & OCF0A)) {}
		TIFR = OCF0A;

		// Читаем значение эталонного источника 1.1 вольт
		uint16_t adc1100 = read_filtered_adc_value(0b1100, true);

		// Получаем текущее значение напряжения питания
		uint16_t adc_ref_voltage = (int32_t)MaxAdc * 1100L / (int32_t)adc1100;

		// Читаем значение АЦП с курка
		uint16_t adc_value = read_filtered_adc_value(AdcChan, false);

		// Переводим значение с курка в милливольты
		uint16_t in_voltage = adc_to_voltage(adc_value, adc_ref_voltage);

		// Производим обработку для основного колеса
		process_for_channel(
			in_voltage,
			adc_ref_voltage,
			smooth_voltage1,
			transl_table1,
			sizeof(transl_table1)/sizeof(AdcPwmItem),
			OCR1A
		);

		// Производим обработку для колеса с редуктором
		process_for_channel(
			in_voltage,
			adc_ref_voltage,
			smooth_voltage2,
			transl_table2,
			sizeof(transl_table2)/sizeof(AdcPwmItem),
			OCR1B
		);
	}
}
