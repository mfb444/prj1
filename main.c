#include "mik32_hal_usart.h"
#include "mik32_hal_spi.h"
#include "mik32_hal_pcc.h"
#include "mik32_hal_gpio.h"

#include "mik32_hal_timer16.h"

#include "mik32_hal_irq.h"

#include "stdlib.h"
#include "stdint.h"
#include "opu.h"
#include "xprintf.h"

#define BUFFER_LENGTH   30

#define MaxCodeRate  7  // Максимальное значение кода скорости

extern char Manual;		   // Признак включения режима ручного управления ОПУ

#define CommandBufferLen	30			// Длина буфера команд
int NBuf;						   		// Число символов в буфере приема
char CommandBuffer[CommandBufferLen];	// Буфер команд

int Answer;			// Ответ на команду
char Command;       // Код команды (0 - команды нет)
float ParamCodeMin;   // Параметр команды (минимальный угол сканирования)
float ParamCodeMax;   // Параметр команды (максимальный угол сканирования)
int ParamV;         // Параметр команды (код скорости)
char State_other;  //Состояние ОПУ

//extern char MotorMove;

extern int Side; //Сторона поворота

extern char ButtonLeft;    // Признак нажатия кнопки "Поворот влево"
extern char ButtonRight;   // Признак нажатия кнопки "Поворот вправо"


Timer16_HandleTypeDef htimer16_1;

Timer16_HandleTypeDef htimer16_2;

volatile char usartBuffer[30]; // Буфер для данных USART
volatile uint8_t usartBufferLength = 0;
volatile uint8_t usartBufferIndex = 0;
volatile bool usartReadyToSend = false;

// Коды скоростей ОПУ

volatile uint16_t counter = 0;
volatile uint16_t pulseCount = 1000;
extern volatile float CurrentDegr = 0;

/* Буфер для хранения вводимых данных */
static char buf[BUFFER_LENGTH];
/* Указатель на текущий элемент буфера */
uint8_t buf_pointer;
/* Флаг готовности принятых данных */
volatile bool buf_ready = false;

SPI_HandleTypeDef hspi0;
USART_HandleTypeDef husart0;

void SystemClock_Config(void);
void USART_Init();
void SPI0_Init();
void GPIO_Init();
void processBuffer(char *);
void DegrOutput(float, char);
static void Timer16_1_Init(void);
static void Timer16_2_Init(void);

float transform_input(uint8_t *master_input)
{
    uint16_t combined = (master_input[0] << 8) | master_input[1];
    //uint16_t result = combined & 0x1FFE; 
    uint16_t result = (combined & 0x7FFF)>>1; 


    float angle = result / 16384.0 * 360.0;

    return angle;
}

int main()
{
    NBuf= 0;	   	  // Буфер команд пуст
	Answer= ANS_NONE; // Ответ не требуется
	Command= 0;		  // Команды нет

    //CurrentDegr = 0;

	ButtonInit();     // Инициализация кнопок и переключателя
    CtrlOPUInit();    // Инициализация привода ОПУ
    StepMotorInit();

    HAL_Init();
    SystemClock_Config();

    USART_Init();
    SPI0_Init();
    GPIO_Init();

    HAL_GPIO_WritePin(GPIO_1, GPIO_PIN_0, GPIO_PIN_HIGH); //MANUAL TEST

    Timer16_1_Init();

    Timer16_2_Init();

    HAL_EPIC_MaskLevelSet(HAL_EPIC_TIMER16_1_MASK | HAL_EPIC_TIMER16_2_MASK); 

    __HAL_PCC_EPIC_CLK_ENABLE();
    HAL_EPIC_MaskLevelSet(HAL_EPIC_UART_0_MASK); 

    HAL_IRQ_EnableInterrupts();

    HAL_Timer16_Counter_Start_IT(&htimer16_2, 1200); //1200 или 30000 для дебага

    while (1)
    {   
        HAL_USART_RXNE_EnableInterrupt(&husart0);
        buf_pointer = 0;
        if(buf_ready)
        {HAL_USART_RXNE_DisableInterrupt(&husart0);
        buf_ready = false;}

        HAL_USART_TXC_EnableInterrupt(&husart0);

        buf_pointer = 1;
        if (usartBuffer[0] != '\0') HAL_USART_WriteByte(&husart0, usartBuffer[0]);

        HAL_DelayMs(20);

        HAL_USART_TXC_DisableInterrupt(&husart0);
    }
}

void processBuffer(char *CommandBuffer)
{
    //Switch_LED();
    // Обработка данных из строки
    if (CommandBuffer[0] != '[')
    {
        // Nbuf = 0;
    }
    else
        switch (CommandBuffer[1])
        {
        case 'P': // --- Пуск сканирования ---
            if (TestDecimal(&CommandBuffer[2], 9) &&
                TestHexdecimal(&CommandBuffer[11], 2) &&
                TestCheckSum(&CommandBuffer[1], 10))
            {
                //Switch_LED();

                float minDegr = 0.0f; // Минимальный угол сканирования
                float maxDegr = 0.0f; // Максимальный угол сканирования

                // Парсинг минимального угла (формат: 4 цифры)
                minDegr = ((CommandBuffer[2] - '0') * 100 + 
                           (CommandBuffer[3] - '0') * 10 + 
                           (CommandBuffer[4] - '0')) +
                          (CommandBuffer[5] - '0') * 0.1f;

                // Парсинг максимального угла (формат: 4 цифры)
                maxDegr = ((CommandBuffer[6] - '0') * 100 + 
                           (CommandBuffer[7] - '0') * 10 + 
                           (CommandBuffer[8] - '0')) +
                          (CommandBuffer[9] - '0') * 0.1f;

                if ((minDegr <= 360.0f) && (maxDegr <= 360.0f) && (minDegr <= maxDegr))
                {
                    //Switch_LED();
                    int Rate; // Код скорости сканирования
                    Rate = CommandBuffer[10] - '0';
                    if ((Rate > 0) && (Rate <= MaxCodeRate))
                    {                // Команда верна, аргументы проверены
                        if (!Manual) // Режим управления от УЭВМ?
                        {            // Да
                            //HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7)

                            ParamCodeMin = minDegr;
                            ParamCodeMax = maxDegr;
                            ParamV = Rate;
                            Command = 'P';
                            Answer = ANS_PY;
                        }
                        else
                        { // Ручной режим
                            Answer = ANS_PM;
                        }
                    }
                    else // Неверно задан код скорости
                        Answer = ANS_PV;
                }
                else // Неверно заданы углы сканирования
                    Answer = ANS_PD;
            }
            else // Неверная структура пакета или контрольная сумма
                Answer = ANS_PN;
            NBuf = 0;
            break;
        case 'L': // --- Пуск поворота влево ---
            if (TestDecimal(&CommandBuffer[2], 1) &&
                TestHexdecimal(&CommandBuffer[3], 2) &&
                TestCheckSum(&CommandBuffer[1], 2))
            {
                int Rate; // Код скорости сканирования
                Rate = CommandBuffer[2] - '0';
                if ((Rate > 0) && (Rate <= MaxCodeRate))
                {                // Команда верна, аргументы проверены
                    if (!Manual) // Режим управления от УЭВМ?
                    {            // Да
                        ParamCodeMin = MINCODE;
                        ParamCodeMax = MAXCODE;
                        ParamV = Rate;
                        Command = 'L';
                        Answer = ANS_LY;
                    }
                    else
                    { // Ручной режим
                        Answer = ANS_LM;
                    }
                }
                else // Неверно задан код скорости
                    Answer = ANS_LV;
            }
            else // Неверная структура пакета или контрольная сумма
                Answer = ANS_LN;
            NBuf = 0;
            break;
        case 'R': // --- Пуск поворота вправо ---
            if (TestDecimal(&CommandBuffer[2], 1) &&
                TestHexdecimal(&CommandBuffer[3], 2) &&
                TestCheckSum(&CommandBuffer[1], 2))
            {
                int Rate; // Код скорости сканирования
                Rate = CommandBuffer[2] - '0';
                if ((Rate > 0) && (Rate <= MaxCodeRate))
                {                // Команда верна, аргументы проверены
                    if (!Manual) // Режим управления от УЭВМ?
                    {            // Да
                        ParamCodeMin = MINCODE;
                        ParamCodeMax = MAXCODE;
                        ParamV = Rate;
                        Command = 'R';
                        Answer = ANS_RY;
                    }
                    else
                    { // Ручной режим
                        Answer = ANS_RM;
                    }
                }
                else // Неверно задан код скорости
                    Answer = ANS_RV;
            }
            else // Неверная структура пакета или контрольная сумма
                Answer = ANS_RN;
            NBuf = 0;
            break;
        case 'S': // --- Стоп ---
            if (TestHexdecimal(&CommandBuffer[2], 2) &&
                TestCheckSum(&CommandBuffer[1], 1))
            {                // Команда верна, аргументы проверены
                if (!Manual) // Режим управления от УЭВМ?
                //if (TRUE)
                {            // Да
                    //HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7);
                    Command = 'S';
                    Answer = ANS_SY;
                }
                else
                { // Ручной режим
                   Answer = ANS_SM;
                }
            }
            else // Неверная структура пакета
                Answer = ANS_SN;
            NBuf = 0;
            break;
        default: // --- Неизвестная команда ---
            Answer = ANS_U;
            NBuf = 0;
            break;
        }
}

void USART_Transmit(char* Buffer){
    HAL_USART_Transmit(&husart0, Buffer, USART_TIMEOUT_DEFAULT);
}

void USART_Print(char* Buffer){
    HAL_USART_Print(&husart0, Buffer, USART_TIMEOUT_DEFAULT);
}

void Start_Motor(uint16_t Period, uint16_t Compare){
    HAL_Timer16_StartPWM_IT(&htimer16_1, Period, Compare);
}

void Stop_Motor(void){
    HAL_Timer16_Stop_IT(&htimer16_1);
    htimer16_1.Instance->CNT = 0;
}

void Switch_LED(void){
    HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7);
}

/*char GetState(void){
    uint8_t pin4 = 0;
    uint8_t pin5 = 1;
    uint8_t inverted_pin4 = pin4 ? 0 : 1;
    uint8_t inverted_pin5 = pin5 ? 0 : 1;
    uint8_t result = ((inverted_pin5 << 1) | inverted_pin4) & 0x03; // Старший бит - PIN5, младший - PIN4
    return result | 0x08; // Добавляем бит 3
}*/   

void DegrOutput(float Degr, char State)
{
  char d;			 // Цифра угла или состояния
  char ChckSum= 'X'; // Контрольная сумма

  int bufIndex = 0;      // Индекс для заполнения буфера
  
  // Преобразование float в int для целой и округленной дробной части
  int intPart = (int)Degr;  // Целая часть
  int decimalPart = (int)((Degr - intPart) * 10 + 0.5); // Округленные десятые доли
  
  // Проверка на случай, если округление даёт 10 десятых (например, 27.95 -> 28.0)
  if (decimalPart == 10) {
      decimalPart = 0;
      intPart += 1;
  }
  
  // Начало пакета данных
  //USART_Print("[");
  // Тип пакета - текущий угол
  //USART_Print("X");

  usartBuffer[bufIndex++] = '[';
  usartBuffer[bufIndex++] = 'X';
  
  // Первая цифра угла (сотни градусов)
  d = intPart / 100;  
  intPart -= d * 100;  
  d += '0';  
  ChckSum ^= d;
  //USART_Transmit(d);
  usartBuffer[bufIndex++] = d;

  // Вторая цифра угла (десятки градусов)
  d = intPart / 10;  
  intPart -= d * 10;  
  d += '0';  
  ChckSum ^= d;
  //USART_Transmit(d);
  usartBuffer[bufIndex++] = d;

  // Третья цифра угла (единицы градусов)
  d = intPart;  
  d += '0';  
  ChckSum ^= d;
  usartBuffer[bufIndex++] = d;

  // Четвертая цифра угла (десятые доли градуса)
  d = decimalPart + '0';  
  ChckSum ^= d;
  usartBuffer[bufIndex++] = d;
  
  // Признаки состояния
  d = ByteToHex(State);  
  ChckSum ^= d;
  usartBuffer[bufIndex++] = d;

  usartBuffer[bufIndex++] = ByteToHex(ChckSum >> 4);
  usartBuffer[bufIndex++] = ByteToHex(ChckSum & 0x0F);
  
  // Конец пакета данных
  usartBuffer[bufIndex++] = ']';

  usartBufferLength = bufIndex;
  usartBufferIndex = 0;
  usartReadyToSend = true;
}

void CountDegr(void){
    //Switch_LED();
    switch (Command)
    {
    case 'L':
        CurrentDegr -= 0.1;
        break;
    
    case 'R':
        CurrentDegr += 0.1;
        break;
    case 'P':
        if (Side==1){
            CurrentDegr += 0.1;
        }
        else{
            CurrentDegr -= 0.1;
        }
    }
}

void SystemClock_Config(void)
{
    PCC_InitTypeDef PCC_OscInit = {0};

    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 128;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);
}

void GPIO_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_PCC_GPIO_0_CLK_ENABLE();
    __HAL_PCC_GPIO_1_CLK_ENABLE();
    __HAL_PCC_GPIO_2_CLK_ENABLE();

    //Дебаг

    GPIO_InitStruct.Pin = GPIO_PIN_7; //Светодиод на плате Elbear
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
    HAL_GPIO_Init(GPIO_2, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6; //Пользовательская кнопка Elbear
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_INPUT;
    HAL_GPIO_Init(GPIO_2, &GPIO_InitStruct);

    //Двигатель

	GPIO_InitStruct.Pin = GPIO_PIN_10; //STEP
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9; //DIR
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8; //SLEEP
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7; //EN
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    //Датчик ЛИР

    //GPIO_InitStruct.Pin = GPIO_PIN_0; //MISO0
    //GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    //HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    //GPIO_InitStruct.Pin = GPIO_PIN_2; //SCLK0
    //GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    //HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    //GPIO_InitStruct.Pin = GPIO_PIN_3; //CS
    //GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    //HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);

    //Датчики угла поворота

    GPIO_InitStruct.Pin = GPIO_PIN_8; //КВ+
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    HAL_GPIO_Init(GPIO_1, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9; //КВ-
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    HAL_GPIO_Init(GPIO_1, &GPIO_InitStruct);

    //Кнопки

    GPIO_InitStruct.Pin = GPIO_PIN_2; //Вправо кнопка
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_INPUT;
    HAL_GPIO_Init(GPIO_1, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1; //Влево кнопка
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_INPUT;
    HAL_GPIO_Init(GPIO_1, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0; //Ручное управление
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_INPUT;
    GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
    HAL_GPIO_Init(GPIO_1, &GPIO_InitStruct);
}

void USART_Init()
{
    husart0.Instance = UART_0;
    husart0.transmitting = Enable;
    husart0.receiving = Enable;
    husart0.frame = Frame_9bit; //Frame_9bit
    husart0.parity_bit = Enable; //должно быть включено
    husart0.parity_bit_inversion = Disable;
    husart0.bit_direction = LSB_First;
    husart0.data_inversion = Disable;
    husart0.tx_inversion = Disable;
    husart0.rx_inversion = Disable;
    husart0.swap = Disable;
    husart0.lbm = Disable;
    husart0.stop_bit = StopBit_1;
    husart0.mode = Asynchronous_Mode;
    husart0.xck_mode = XCK_Mode3;
    husart0.last_byte_clock = Disable;
    husart0.overwrite = Disable;
    husart0.rts_mode = AlwaysEnable_mode;
    husart0.dma_tx_request = Disable;
    husart0.dma_rx_request = Disable;
    husart0.channel_mode = Duplex_Mode;
    husart0.tx_break_mode = Disable;
    husart0.Interrupt.ctsie = Disable;
    husart0.Interrupt.eie = Disable;
    husart0.Interrupt.idleie = Disable;
    husart0.Interrupt.lbdie = Disable;
    husart0.Interrupt.peie = Disable;
    husart0.Interrupt.rxneie = Disable;
    husart0.Interrupt.tcie = Disable;
    husart0.Interrupt.txeie = Disable;
    husart0.Modem.rts = Disable;  // out
    husart0.Modem.cts = Disable;  // in
    husart0.Modem.dtr = Disable;  // out
    husart0.Modem.dcd = Disable;  // in
    husart0.Modem.dsr = Disable;  // in
    husart0.Modem.ri = Disable;   // in
    husart0.Modem.ddis = Disable; // out
    husart0.baudrate = 19200; //19200
    HAL_USART_Init(&husart0);
}

void SPI0_Init(void)
{
    hspi0.Instance = SPI_0;

    /* Режим SPI */
    hspi0.Init.SPI_Mode = HAL_SPI_MODE_MASTER;

    /* Настройки */
    hspi0.Init.CLKPhase = SPI_PHASE_OFF;
    hspi0.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi0.Init.ThresholdTX = 4;

    /* Настройки для ведущего */
    hspi0.Init.BaudRateDiv = SPI_BAUDRATE_DIV32;
    hspi0.Init.Decoder = SPI_DECODER_NONE;
    hspi0.Init.ManualCS = SPI_MANUALCS_OFF;
    hspi0.Init.ChipSelect = SPI_CS_0;      

    if (HAL_SPI_Init(&hspi0) != HAL_OK);
}

static void Timer16_1_Init(void)
{
    htimer16_1.Instance = TIMER16_1;

    /* Настройка тактирования */
    htimer16_1.Clock.Source = TIMER16_SOURCE_INTERNAL_OSC32M;
    htimer16_1.CountMode = TIMER16_COUNTMODE_INTERNAL;  /* При тактировании от Input1 не имеет значения */
    htimer16_1.Clock.Prescaler = TIMER16_PRESCALER_16;
    htimer16_1.ActiveEdge = TIMER16_ACTIVEEDGE_RISING;  /* Выбирается при тактировании от Input1 */

    /* Настройка режима обновления регистра ARR и CMP */
    htimer16_1.Preload = TIMER16_PRELOAD_AFTERWRITE;

    /* Настройка триггера */
    //htimer16_1.Trigger.Source = TIMER16_TRIGGER_TIM1_GPIO1_9; 
    htimer16_1.Trigger.ActiveEdge = TIMER16_TRIGGER_ACTIVEEDGE_SOFTWARE;    /* При использовании триггера значение должно быть отлично от software */
    //htimer16_1.Trigger.TimeOut = TIMER16_TIMEOUT_DISABLE;   /* Разрешить повторное срабатывание триггера */

    /* Настройки фильтра */
    htimer16_1.Filter.ExternalClock = TIMER16_FILTER_NONE;
    htimer16_1.Filter.Trigger = TIMER16_FILTER_NONE;

    /* Настройка режима энкодера */

    htimer16_1.EncoderMode = TIMER16_ENCODER_DISABLE;

    /* Выходной сигнал */
    htimer16_1.Waveform.Enable = TIMER16_WAVEFORM_GENERATION_ENABLE;
    htimer16_1.Waveform.Polarity = TIMER16_WAVEFORM_POLARITY_NONINVERTED;

    HAL_Timer16_Init(&htimer16_1);
}

static void Timer16_2_Init(void)
{
    htimer16_2.Instance = TIMER16_2;

    /* Настройка тактирования */
    htimer16_2.Clock.Source = TIMER16_SOURCE_INTERNAL_OSC32K;
    htimer16_2.CountMode = TIMER16_COUNTMODE_INTERNAL;  /* При тактировании от Input1 не имеет значения */
    htimer16_2.Clock.Prescaler = TIMER16_PRESCALER_1;
    htimer16_2.ActiveEdge = TIMER16_ACTIVEEDGE_RISING;  /* Выбирается при тактировании от Input1 */

    /* Настройка режима обновления регистра ARR и CMP */
    htimer16_2.Preload = TIMER16_PRELOAD_AFTERWRITE;

    /* Настройка триггера */
    //htimer16_1.Trigger.Source = TIMER16_TRIGGER_TIM1_GPIO1_9; 
    htimer16_2.Trigger.ActiveEdge = TIMER16_TRIGGER_ACTIVEEDGE_SOFTWARE;    /* При использовании триггера значение должно быть отлично от software */
    //htimer16_1.Trigger.TimeOut = TIMER16_TIMEOUT_DISABLE;   /* Разрешить повторное срабатывание триггера */

    /* Настройки фильтра */
    htimer16_2.Filter.ExternalClock = TIMER16_FILTER_NONE;
    htimer16_2.Filter.Trigger = TIMER16_FILTER_NONE;

    /* Настройка режима энкодера */

    htimer16_2.EncoderMode = TIMER16_ENCODER_DISABLE;

    /* Выходной сигнал */
    htimer16_2.Waveform.Enable = TIMER16_WAVEFORM_GENERATION_ENABLE;
    htimer16_2.Waveform.Polarity = TIMER16_WAVEFORM_POLARITY_NONINVERTED;

    HAL_Timer16_Init(&htimer16_2);
}

void trap_handler()
{
    if (EPIC_CHECK_TIMER16_1())
    {
        uint32_t interrupt_status = HAL_Timer16_GetInterruptStatus(&htimer16_1);

        if (interrupt_status & TIMER16_ISR_ARR_MATCH_M)
        {
        
        } 
        HAL_Timer16_ClearInterruptMask(&htimer16_1, 0xFFFFFFFF); /* Сброс нескольких флагов прерываний по маске */
    }

    if (EPIC_CHECK_TIMER16_2())
    {
        uint32_t interrupt_status = HAL_Timer16_GetInterruptStatus(&htimer16_2);

        if (interrupt_status & TIMER16_ISR_ARR_MATCH_M)
        {
            uint8_t master_output[] = {0xA0, 0xA1};
            uint8_t master_input[sizeof(master_output)];

            //HAL_StatusTypeDef SPI_Status = HAL_SPI_Exchange(&hspi0, master_output, master_input, sizeof(master_output), SPI_TIMEOUT_DEFAULT);

            HAL_SPI_Exchange(&hspi0, master_output, master_input, sizeof(master_output), SPI_TIMEOUT_DEFAULT);

            //if (SPI_Status != HAL_OK)
            //{
            //    HAL_SPI_ClearError(&hspi0);
            //}
            
            CurrentDegr = transform_input(master_input);

            State_other = GetState();
            KV();
            DegrOutput(CurrentDegr, State_other);
            if(Answer != ANS_NONE){
                //AnswerOutput(Answer);
                Answer = ANS_NONE;
            }

            // Чтение состояния кнопок и переключателя
                char Chng= GetButton();
                if(Chng & 1)
                { // Было изменение режима работы: "Ручной"/"Автомат"
            //	   AnswerOutput(Manual ? ANS_ZH : ANS_ZW);  // Передать сообщение об изменении режима
                MakeCommand('S', MINCODE, MAXCODE, 0);   // Привод - "Стоп"
                }
                if(State_other & 0x04)  // Перегрузка привода?
                MakeCommand('S', MINCODE, MAXCODE, 0); // Да, перегрузка: Привод - "Стоп"
                else if(Manual)  // Ручной режим управления?
                { // Да
                if(Chng & 0x6)  // Было изменение состояния кнопок?
                { // Да
                    if(ButtonLeft && !ButtonRight){ // Нажата только "Влево"?
                        Command = 'L';
                        MakeCommand('L', MINCODE, MAXCODE, VMANUAL);  // Привод - "Влево"
                    }
                    else if(!ButtonLeft && ButtonRight){ // Нажата только "Вправо"
                         Command = 'R';
                        MakeCommand('R', MINCODE, MAXCODE, VMANUAL); // Привод - "Вправо"
                    }
                    else{ // Не нажаты кнопки, или нажаты обе
                        Command = 'S';
                        MakeCommand('S', MINCODE, MAXCODE, 0); // Привод - "Стоп"
                    }
                }
                }
            if (!Manual){
                MakeCommand(Command, ParamCodeMin, ParamCodeMax, ParamV);
            }
            CtrlMake();
        } 
        HAL_Timer16_ClearInterruptMask(&htimer16_2, 0xFFFFFFFF); /* Сброс нескольких флагов прерываний по маске */
    }

    if (EPIC_CHECK_UART_0())
    {
        /* Прием данных: запись в буфер */
        if (HAL_USART_RXNE_ReadFlag(&husart0))
        {
            buf[buf_pointer] = HAL_USART_ReadByte(&husart0);
            /* Если принят символ '\n', заменить его на '\0' (символ конца строки) */
            if (buf[buf_pointer] == ']')
            {
                buf[buf_pointer] = '\0';
                buf_pointer = 0;
                processBuffer(buf);
                buf_ready = true;
            }
            else
            {
                buf_pointer += 1;
                if (buf_pointer >= BUFFER_LENGTH) buf_pointer = 0;
                buf_ready = false;
            }
            HAL_USART_RXNE_ClearFlag(&husart0);
        }

        /* Передача данных: чтение из буфера */
        if (HAL_USART_TXC_ReadFlag(&husart0))
        {
            if (usartBuffer[buf_pointer] != '\0')
            {
                HAL_USART_WriteByte(&husart0, usartBuffer[buf_pointer]);
                buf_pointer += 1;
            }
            else buf_pointer = 0;
            HAL_USART_TXC_ClearFlag(&husart0);
        }
    }

    /* Сброс прерываний */
    HAL_EPIC_Clear(0xFFFFFFFF);
}