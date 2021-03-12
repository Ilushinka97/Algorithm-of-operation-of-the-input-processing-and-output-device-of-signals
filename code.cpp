#include <ADuC842.h>
#include <vect.h>
#include <math.h>
#include <DAC.h>
#include <max.h>

#define ID 0x00
#define function_code 0x03

#define Address_of_the_first_register_HIGH 0x00		//   произвольный адрес первого
#define Address_of_the_first_register_LOW  0x05	 	// 			 		регистра		 		

#define Number_of_registers_HIGH 0x00   //  количество
#define Number_of_registers_LOW  0x02  //   регисторов



// ----- Объявление переменных ----- //

unsigned short Meander_frequency = 0; 
unsigned short counter_m = 0;
unsigned short shim=0;

unsigned char in_my_uart = 0; 
unsigned char	send[9]; 
unsigned char buf[8];
unsigned char receive = 0;
unsigned char transmit = 0; 
unsigned char count_u = 0;

unsigned char in_my_uart1 = 0;
unsigned char var = 0;
unsigned char mul = 1;
unsigned char count1 = 0;
unsigned char message_length = 0;


bit flag = 1;
bit got = 0; 
bit start = 1; 
bit myTI = 0; 
bit modbus_error = 0; 
bit CRC_error = 0;


float napr_mc=0;


unsigned char modbus_input_check[6] = {
	ID, //0								
	function_code, //1
	Address_of_the_first_register_HIGH,  //2
	Address_of_the_first_register_LOW, //3
	Number_of_registers_HIGH, //4	
	Number_of_registers_LOW,  //5 
};
								  

																			
// ----- Расчет контрольной суммы ----- //

unsigned short CRC16 (unsigned char *input, unsigned short kk)
{
	unsigned short crc = 0xFFFF; //65535 = 1111 1111 1111 1111
	unsigned short p = 0, i = 0;
	for (p=0; p<kk; p++)
	{
		crc^=(unsigned short)input[p];  //xor s prisvoeniem
		for (i=8; i!=0; i--)
		{	
			if ((crc&0x0001)!=0)  //proverka mladshego bita na ==1
			{
				crc>>=1;
				crc^=0xA001;
			}
			else {
				crc>>=1;
			}
			
		}
	}

	return crc; // mladshii i starshii byte pomenyani mestami
}

																			


// ----- Обработка полученного пакета по протоколу MODBUS RTU ----- //
 
void MODBUS_RTU(void)		
{
	short i = 0;
	//----- Расчет контрольной суммы, если не сошлась modbus_error = 1

	unsigned short CRC_in = 0, CRC_out = 0;
	unsigned char CRC_in_Hi = 0, CRC_in_Low = 0;

	modbus_error = 0;
	CRC_in = CRC16(buf,6);

	CRC_in_Hi = (CRC_in&0x00FF);
	CRC_in_Low = CRC_in>>8;

	if (CRC_in_Hi != buf[6] || CRC_in_Low != buf[7])
	{
		modbus_error = 1;
		CRC_error = 1;	
	}
	
	
	if(!CRC_error)
	{
	if (buf[3] == (Address_of_the_first_register_LOW +1) && buf[5] == Number_of_registers_LOW)
	{
		modbus_error = 1;
	}

	if(!modbus_error)
	{
		for(i = 0; i<6; i++)
		{

			if (i == 3)	 // адрес 
			{
				if (buf[i] != modbus_input_check[i] && buf[i] != (modbus_input_check[i] + 1) )
				{
					modbus_error = 1;
					break;	
				}
			}

			if (i == 5)	// количество байт 
			{
				if (buf[i] != modbus_input_check[i] && buf[i] != (modbus_input_check[i] - 1))
				{
					modbus_error = 1;
					break;	
				}
			}
			if (i!=3 && i!=5)
			{
				if (buf[i] != modbus_input_check[i])
				{
					modbus_error = 1;
					break;
				}  
			}		
		}
	}
	}

	 if (modbus_error)
	 {
		 // Сообщение об ошибке
		 message_length = 5;

		 if(!CRC_error)
		 {
			 if (i==1) // не соответствует код функции.	error_code = 1
			 {
			  send[0] = ID;
				send[1] = (buf[1] | 0x80);
				send[2] = 0x01; 
				CRC_out = CRC16(send,3);
				send[3] = CRC_out; 
				send[4] = CRC_out>>8;
			 } 
			 if (i==2 || i==3) // не соответствует адрес. error_code = 2
			 {
			 	send[0] = ID;
				send[1] = (buf[1] | 0x80);
				send[2] = 0x02; 
				CRC_out = CRC16(send,3);
				send[3] = CRC_out;
				send[4] = CRC_out>>8;
			 }	
			 if (i==4 || i==5)   // не соответствует колиество данных. error_code = 3
			 {
			 	send[0] = ID;
				send[1] = (buf[1] | 0x80);
				send[2] = 0x03; 
				CRC_out = CRC16(send,3);
				send[3] = CRC_out; 
				send[4] = CRC_out>>8;	
			 } 
		 }else   // не правильная CRC
		 {
		 		send[0] = ID;
				send[1] = (buf[1] | 0x80);
				send[2] = 0xFF; 
				CRC_out = CRC16(send,3);
				send[3] = CRC_out; 
				send[4] = CRC_out>>8;	
		 }

	 } 
	 
	 else
	 {
		 // Отправка данных 
	 	send[0] = ID;
		send[1] = function_code;

		if(buf[3] == Address_of_the_first_register_LOW)
		{
			if(buf[5] == Number_of_registers_LOW)
			{
				send[2] = 4;  // кол-во байт 
				send[3] = Meander_frequency>>8; 
				send[4] = Meander_frequency; 
				send[5] = 0x00;
				send[6] = in_my_uart1;
				CRC_out = CRC16(send,7);
				send[7] = CRC_out;  // crc
				send[8] = CRC_out>>8; 	// crc
				message_length = 9;
			}else{
				send[2] = 2;  // кол-во байт
				send[3] = Meander_frequency>>8;	
				send[4] = Meander_frequency;
				CRC_out = CRC16(send,5);
				send[5] = CRC_out; // crc
				send[6] = CRC_out>>8; // crc
				message_length = 7;
			} 
		} else{
			 send[2] = 2;  // кол-во байт
			 send[3] = 0x00;
			 send[4] = in_my_uart1;
			 CRC_out = CRC16(send,5);
			 send[5] = CRC_out;   // crc
			 send[6] = CRC_out>>8;  // crc
			 message_length = 7;	
		}		
	 } 	

	 
	SBUF = send[0];

}



// ----- Обработчик прерываия таймер 0. Программный UART ----- //


 void T0_Interrupt (void) interrupt 1
 {
	TH0 = 0xED; // предустановка на 300 бит/с 	
	TL0 = 0xCA;  
   
   // UART на выводе P1.7 
	var = (P1 & 0x80);
	 
	if(start == 1 )
	{
		count_u++;
		
		if (var == 0x80)
		{
			 in_my_uart +=mul;
		}
		mul *= 2;

		if(count_u == 8)
		{
			count_u = 0;
			start = 0;
			myTI = 1;
			mul = 1;
			TR0 = 0;
			ET0 = 0;
		}
			
	}

	if(var == 0 && count_u == 0 && myTI == 0 )
	{
	
		start = 1;
	
	}	 
 }




// ----- Обработчик внешнего прерывания INT0. Подсчет частоты вх. меандра ----- //

 void Vnesh0(void) interrupt 0
 {
		counter_m++; 		
 }


 
// ----- Обработчик внешнего прерывания INT1. Синхронизаця с сетью ----- //

 void Vnesh1(void) interrupt 2
 {
		count1++;
		if (count1 == 50)	
		{
				Meander_frequency = counter_m;
				if (Meander_frequency<=5000)
				{
				Meander_frequency = 0;	
				}
				counter_m = 0;
				count1 = 0;
			
				shim = (unsigned short)((Meander_frequency*0.09322));				   
				PWM0L = shim;
				PWM0H = shim>>8;	  
		}																				   
 }

	


 // ----- Обработчик прерывания от UART ----- //

void UART(void) interrupt 4
{
	if (flag == 1) 
	{ 
 		if(RI == 1) 
 		{ 	
 			RI = 0; 
 			buf[receive] = SBUF; 
 			receive++; 
			if (receive == 8) 
			{
				if (buf[0] != ID)
				{
					receive = 0;
				}
				else
				{		
					flag = 0;
					receive = 0;
					got = 1;
				}
			}
		} 
	}

	if (TI == 1)
	{
		TI = 0;
		if (flag==0) 
		{
			transmit++;
			SBUF = send[transmit];
		}
		if(transmit >= (message_length-1))
		{
			transmit = 0;
			flag = 1;
		}	
	}
}






void main(void)
{
	SetVect(1,T0_Interrupt);
	SetVect(0,Vnesh0);	
	SetVect(2,Vnesh1);
	SetVect(4,UART);

	PLLCON = 0x02;  // Регистр фазовой автоподстройки частоты. Делитель частоты = 1

	//-----------Настройка UART ---------------
	SCON = 0x40;
	REN = 1;  // Разрешение приема
	
        // ------ Настройка таймера 2
	RCLK = 1;
	TCLK = 1; // Тактовые импульсы от таймера 2 
	RCAP2H = 0xFF;
	RCAP2L = 0x93;  // Скорость 9600 бит/с
	TR2 = 1; // Запуск таймера 2
	ES = 1; // Разрешение прерываний от UART
	EA = 1;
	//------------------------------------------
	TI=0;
	RI=0;
 
	TMOD = 0x11;

	ET0 = 1;
	
	EX0 = 1;
	IT0 = 1;
	
	EX1 = 1;
	IT1 = 1;

	// T0 настроен на скорость программного UART в 300 бит/с. Переполняется 1 раз в (1/300) сек 
	TH0 = 0xED;
	TL0 = 0xCA;
	
	TR0 = 1;


// ----- настройка DAC0
	SwitchDAC(0,1);
	InitDAC(0);

 // ----- настройка ШИМ
	CFG842 = 0x00; // для ШИМ испоьзуются выводы P2.6 и P2.7
	PWMCON = 0x13; // 0001 0011 - mode 1, исходная частота 16,78 МГц

	PWM1L =	0x76;
	PWM0L = 0x00;  // 0x0576 - частота 12 кГц ;
	PWM1H =	0x05;  // 0x0000 - начальная длительность импульса = 0
	PWM0H =	0x00;
	

	P1 = 0x00;  // P1 в режиме цифрового входа
	
										  
	while (1)
	{
		if(myTI)
		{
			myTI = 0;
		
			in_my_uart1 = in_my_uart;
			napr_mc = (float)(in_my_uart*((-1.35)/180)+5);
			SetVoltage(napr_mc,0);
			in_my_uart = 0;
			TH0 = 0xED; // Загрузка в T0 значений для скорости 300 бит/с
			TL0 = 0xCA;
			ET0 = 1;
			TR0 = 1;	 		
		}
		
																				
		
		if (got)
		{
			got = 0;
			MODBUS_RTU();		
		}
		
	}
	
}



