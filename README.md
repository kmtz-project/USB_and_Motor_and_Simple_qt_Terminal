# USB_and_Motor_and_Simple_qt_Terminal
Программа для управления шаговым двигателем и работы с энкодером (maple mini clone stm32). 

Проект собран в system workbench for stm32. 

Используется STM32F1xx_HAL_Driver, CMSIS, не включенные в данный архив.

_____________________________________________________________________________________________________________________________

Используемые "ноги" микроконтроллера

// PINA for Motor control

N_ENABLE  GPIO_PIN_1

MS1       GPIO_PIN_2

MS2       GPIO_PIN_3

MS3       GPIO_PIN_4

STEP      GPIO_PIN_5

DIR       GPIO_PIN_6


// PINB for EMS22A

PIN_CLK	  GPIO_PIN_10

PIN_DO 	  GPIO_PIN_1

PIN_CS 	  GPIO_PIN_0


// PINB leds

LED1_YELLOW_PIN   GPIO_PIN_1

LED2_BLUE_PIN     GPIO_PIN_0


// PINA led

LED3_GREEN_PIN    GPIO_PIN_7


Входящие команды для микроконтроллера:

    1.  0 1 0 0 0 0 0 0 (0x80) - данные от энкодера

    2.  0 0 x x x x x x - данные для управления двигателем
   	    | | | | | |
   	    | | | | | DIR
   	    | | | | STEP
   	    | | | MS3
   	    | | MS2
   	    | MS1
  	    ENABLE

    3. 0 1 x x x x x x - данные для управления двигателем + данные от энкодера

    4. 1 0 0 0 0 x x x - управление светодиодами (1 - ON, 0 - OFF)                                          
	  	 | | |
		 | | LED1
		 | LED2
		 LED3
	    
Ответные команды:
    1. От энкодера максимальное значение 1024 - 11 бит, то есть два байта по UART: [0000 0XXX] [XXXX XXXX]

_____________________________________________________________________________________________________________________________


А также простейший терминал для отладки с ПК (Qt).


_____________________________________________________________________________________________________________________________



Общий алгоритм работы системы

1 После подключения питания микроконтроллер ожидает сообщения '0'по USB как признак начала работы с "верхним" уровнем.

2 Микроконтроллер выдаёт "наверх" информационные сообщения по USB:

"Write 1 for stepper motor driver"

"Write 2 for encoder position"

3.1 В случае получения '1' - микроконтроллер выдаст "N_ENABLE MS1 MS2 MS3 STEP DIR: " и будет ожидать с "верхнего" уровня 6 бит 
для управления шаговым двигателем. После окончания перемещения шагового двигателя микроконтроллер выдаст на "верхний" уровень 
данные с энкодера.

3.2 В случае получения '2' - микроконтроллер выдаст данные с энкодера (от 0 до 1024).

4 Возврат к пункту 2.
