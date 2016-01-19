#include "PCA9685.h"

/**	основные пользовательские функции **/
//		инициализация чипа
void	PCA9685::begin(uint8_t i, uint16_t j){
		PCA_uint_ID=i; PCA_uint_PWM=j; PCA_func_SET_MODE(); digitalWrite(ALL,LOW);							//	сохраняем адрес чипа и частоту ШИМ, устанавливаем регистры режимов работы, устанавливаем все выходы в логические «0»
}
//		установка ШИМ на выходе
void	PCA9685::analogWrite(uint8_t i, uint16_t j, uint16_t k){
		if(i>=16&&i!=ALL){return;}																			//	игнорируем команду
		if(j==0)  {digitalWrite(i,LOW ); return;}															//	устанавливаем логический 0
		if(j>4095){digitalWrite(i,HIGH); return;}															//	устанавливаем логическую 1 
		if(k>4095){k=4095;}	j+=k; if(j>4095){j-=4096;}														//	отодвигаем спад импульса в зависимости от фазового сдвига
		BUS_buff_DATA[0]=i==ALL?0xFA:(i*4+6);																//	определяем адрес первого регистра для записи данных
		BUS_buff_DATA[1]=k; BUS_buff_DATA[2]=k>>8; BUS_buff_DATA[3]=j; BUS_buff_DATA[4]=j>>8;				//	определяем данные для записи в 4 регистра, начиная с первого определённого
		I2C_func_START();																					//	отправляем сигнал START
		I2C_func_SEND_ID(PCA_uint_ID,0);																	//	отправляем адрес чипа и бит RW=0
		I2C_func_WRITE_WORD(5);																				//	отправляем 5 байт из массива BUS_buff_DATA (1 байт адреса первого регистра и 4 байта данных)
		I2C_func_STOP();																					//	отправляем сигнал STOP
}
//		установка логического уровня на выходе
void	PCA9685::digitalWrite(uint8_t i, bool j){
		if(i>=16&&i!=ALL){return;}																			//	игнорируем команду
		BUS_buff_DATA[0]=i==ALL?0xFA:(i*4+6);																//	определяем адрес первого регистра для записи данных
		BUS_buff_DATA[1]=0; BUS_buff_DATA[2]=j?16:0; BUS_buff_DATA[3]=0; BUS_buff_DATA[4]=j?0:16;			//	определяем данные для записи в 4 регистра, начиная с первого определённого
		I2C_func_START();																					//	отправляем сигнал START
		I2C_func_SEND_ID(PCA_uint_ID,0);																	//	отправляем адрес чипа и бит RW=0
		I2C_func_WRITE_WORD(5);																				//	отправляем 5 байт из массива BUS_buff_DATA (1 байт адреса первого регистра и 4 байта данных)
		I2C_func_STOP();																					//	отправляем сигнал STOP
}
//		чтение установленного коэффициента заполнения на выходе
uint16_t PCA9685::analogRead(uint8_t i){
		if(i>=16){return 0;} uint16_t j=0, k=0;																//	игнорируем команду
		I2C_func_START();																					//	отправляем сигнал START
		I2C_func_SEND_ID(PCA_uint_ID,0);																	//	отправляем адрес чипа и бит RW=0
		I2C_func_WRITE_BYTE(i*4+6);																			//	отправляем адрес первого регистра, данные которого нужно прочитать
		I2C_func_RESTART();																					//	отправляем сигнал RESTART
		I2C_func_SEND_ID(PCA_uint_ID,1);																	//	отправляем адрес чипа и бит RW=1
		I2C_func_READ_WORD(4);																				//	читаем 4 байта в массив BUS_buff_DATA
		I2C_func_STOP();																					//	отправляем сигнал STOP
		if(BUS_buff_DATA[3]&0x10){return 0;}																//	на читаемом выходе установлен 0
		if(BUS_buff_DATA[1]&0x10){return 4096;}																//	на читаемом выходе установлена 1
		k=(BUS_buff_DATA[1]<<8)+BUS_buff_DATA[0];															//	количество тактов до фронта импульса
		j=(BUS_buff_DATA[3]<<8)+BUS_buff_DATA[2];															//	количество тактов до спада импульса
		if(j<k){j+=4096;} j-=k; return j;																	//	выводим коэффициент заполнения с учётом фазового сдвига
}
//		установка угла поворота сервопривода
void	PCA9685::servoWrite(uint8_t i, uint16_t j){
		uint8_t k=i; if(i==ALL){i=15; k=0;} if(i>=16||j>360){return;} 										//	игнорируем команду
		for(int n=k; n<=i; n++){analogWrite(n,map((j<=PCA_uint_ANGLE[n]?j:PCA_uint_ANGLE[n]),0,PCA_uint_ANGLE[n],PCA_uint_ANGLE_MIN[n],PCA_uint_ANGLE_MAX[n]));}
}
//		установка параметров для сервопривода
void	PCA9685::servoSet(uint8_t i, uint16_t j, uint16_t a, uint16_t b){
		uint8_t k=i; if(i==ALL){i=15; k=0;} if(i>=16||j==0||j>360||a>4095||b>4095||b==0){return;} 			//	игнорируем команду
		for(int n=k; n<=i; n++){PCA_uint_ANGLE[n]=j; PCA_uint_ANGLE_MIN[n]=a; PCA_uint_ANGLE_MAX[n]=b;}		//	применяем значения максимального угла поворота и коэффициентов заполнения для крайних положений сервопривода
}

/**	дополнительные пользовательские функции **/
//		инверсия сигналов на всех выходах
void	PCA9685::invert(bool i){PCA_flag_INVRT=i; PCA_func_SET_MODE();}										//	установка или сброс флага INVRT
//		выбор интегрированной схемы подключения выходов в чипе
void	PCA9685::outdrv(bool i){PCA_flag_OUTDRV=i; PCA_func_SET_MODE();}									//	установка или сброс флага OUTDRV
//		состояние на выходах при подаче на вход OE уровня логической «1»
void	PCA9685::outState(uint8_t i){if(i>2){return;} PCA_uint_OUTNE=i; PCA_func_SET_MODE();}				//	установка двух битов OUTNE
//		установка частоты внешнего источника тактирования
void	PCA9685::extClock(uint16_t i){if(i){PCA_flag_EXTCLK=true; PCA_uint_OSC=i; PCA_uint_OSC*=1000; PCA_func_SET_MODE();}else{PCA_flag_EXTCLK=false; restart(); PCA_uint_OSC=25000000; PCA_func_SET_MODE(); digitalWrite(ALL,LOW);}}
//		перезагрузка всех чипов на шине
void	PCA9685::restart(){I2C_func_START();I2C_func_SEND_ID(0,0);I2C_func_WRITE_BYTE(6);I2C_func_STOP();}	//	отправка команды «SWRST Call»
//		установка скорости шины
void	PCA9685::bus(uint16_t i){I2C_mass_STATUS[0]=i; PCA_flag_I2C=true; I2C_func_begin();}				//	инициализация шины I2C с установкой частоты (в кГц)
//		чтение или запись одного байта данных в регистр
uint8_t	PCA9685::reg(uint8_t i, uint16_t j){I2C_func_START(); I2C_func_SEND_ID(PCA_uint_ID,0); I2C_func_WRITE_BYTE(i); if(j>255){I2C_func_RESTART(); I2C_func_SEND_ID(PCA_uint_ID,1); j=I2C_func_READ_BYTE(0);}else{I2C_func_WRITE_BYTE(j);} I2C_func_STOP(); return j;}

/**	внутренние функции **/
//		установка регистров режимов работы MODE1, MODE2 и PRE_SCALE
void	PCA9685::PCA_func_SET_MODE(){
		if(!PCA_flag_I2C){PCA_flag_I2C=true; I2C_func_begin();}												//	инициализация шины I2C с установкой частоты (если она не инициирована ранее)
		uint16_t j=(PCA_uint_OSC/(4096*uint32_t(PCA_uint_PWM)))-1; if(j<3){j=3;} if(j>255){j=255;}			//	определяем значение предделителя
		reg(0,0x30);																						//	отправляем байт данных в регистр MODE1     (устанавливаем флаги AL и SLEEP, остальные флаги сброшены)
		reg(0xFE,j);																						//	отправляем байт данных в регистр PRE_SCALE (устанавливаем предделитель для частоты ШИМ)
		reg(1,PCA_flag_INVRT<<4|PCA_flag_OUTDRV<<2|PCA_uint_OUTNE);											//	отправляем байт данных в регистр MODE2     (записываем флаги INVRT, OUTDRV, OUTNE, флаг OCH сброшен)
		reg(0,PCA_flag_EXTCLK<<6|0x30);																		//	отправляем байт данных в регистр MODE1     (записываем флаг EXTCLK, флаги AL и SLEEP установлены, остальные флаги сброшены)
		reg(0,0x20);																						//	отправляем байт данных в регистр MODE1     (сбрасываем флаг SLEEP, остальные флаги без изменений) / флаг EXTCLK не сбрасывается записью нуля
		delayMicroseconds(500);																				//	ждём выполнение действий по сбросу флага SLEEP
		reg(0,0xA0);																						//	отправляем байт данных в регистр MODE1     (сбрасываем флаг RESTART, остальные флаги без изменений) / флаг RESTART сбрасывается записью единицы
}

/**	внутренние функции для работы с шиной I2C **/
void	PCA9685::I2C_func_begin			()										{					 I2C_mass_STATUS[2]=1;	TWBR = ((F_CPU/(I2C_mass_STATUS[0]*1000))-16)/2; if(TWBR<10){TWBR=10;} TWSR&=(~(_BV(TWPS1)|_BV(TWPS0)));}
void	PCA9685::I2C_func_START			()										{int I2C_var_I=0;	 I2C_mass_STATUS[2]=1;										TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);			while(!(TWCR & _BV(TWINT))){I2C_var_I++; if(I2C_var_I>I2C_mass_STATUS[1]){I2C_mass_STATUS[2]=0; break;}} I2C_mass_STATUS[3] = TWSR & 0xF8; if(I2C_mass_STATUS[3]!=0x08){I2C_mass_STATUS[2]=0;}}
void	PCA9685::I2C_func_RESTART		()										{int I2C_var_I=0; if(I2C_mass_STATUS[2]){										TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);			while(!(TWCR & _BV(TWINT))){I2C_var_I++; if(I2C_var_I>I2C_mass_STATUS[1]){I2C_mass_STATUS[2]=0; break;}} I2C_mass_STATUS[3] = TWSR & 0xF8; if(I2C_mass_STATUS[3]!=0x10){I2C_mass_STATUS[2]=0;}}}
void	PCA9685::I2C_func_SEND_ID		(uint8_t I2C_byte_ID, bool I2C_bit_RW)	{int I2C_var_I=0; if(I2C_mass_STATUS[2]){	TWDR = (I2C_byte_ID<<1)+I2C_bit_RW;	TWCR = _BV(TWINT) | _BV(TWEN);						while(!(TWCR & _BV(TWINT))){I2C_var_I++; if(I2C_var_I>I2C_mass_STATUS[1]){I2C_mass_STATUS[2]=0; break;}} I2C_mass_STATUS[3] = TWSR & 0xF8; if(I2C_bit_RW) {if(I2C_mass_STATUS[3]!=0x40){I2C_mass_STATUS[2]=0;}}else{if(I2C_mass_STATUS[3]!=0x18){I2C_mass_STATUS[2]=0;}}}}
void	PCA9685::I2C_func_WRITE_BYTE	(uint8_t I2C_byte_DATA)					{int I2C_var_I=0; if(I2C_mass_STATUS[2]){	TWDR = I2C_byte_DATA;				TWCR = _BV(TWINT) | _BV(TWEN);						while(!(TWCR & _BV(TWINT))){I2C_var_I++; if(I2C_var_I>I2C_mass_STATUS[1]){I2C_mass_STATUS[2]=0; break;}} I2C_mass_STATUS[3] = TWSR & 0xF8; if(I2C_mass_STATUS[3]!=0x28){I2C_mass_STATUS[2]=0;}}}
uint8_t	PCA9685::I2C_func_READ_BYTE		(bool I2C_bit_ACK)						{int I2C_var_I=0; if(I2C_mass_STATUS[2]){										TWCR = _BV(TWINT) | _BV(TWEN) | I2C_bit_ACK<<TWEA;	while(!(TWCR & _BV(TWINT))){I2C_var_I++; if(I2C_var_I>I2C_mass_STATUS[1]){I2C_mass_STATUS[2]=0; break;}} I2C_mass_STATUS[3] = TWSR & 0xF8; if(I2C_bit_ACK){if(I2C_mass_STATUS[3]!=0x50){I2C_mass_STATUS[2]=0;}}else{if(I2C_mass_STATUS[3]!=0x58){I2C_mass_STATUS[2]=0;}} return TWDR;}else{return 0xFF;}}
void	PCA9685::I2C_func_STOP			()										{int I2C_var_I=0;																TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);			while(!(TWCR & _BV(TWSTO))){I2C_var_I++; if(I2C_var_I>I2C_mass_STATUS[1]){I2C_mass_STATUS[2]=0; break;}} delayMicroseconds(20);}
void	PCA9685::I2C_func_WRITE_WORD	(uint8_t I2C_byte_SUM)					{int I2C_var_I=0;	 BUS_buff_DATA[0x20]=0; for(I2C_var_I=0; I2C_var_I<I2C_byte_SUM; I2C_var_I++){I2C_func_WRITE_BYTE(BUS_buff_DATA[I2C_var_I]);							 if(I2C_mass_STATUS[2]){BUS_buff_DATA[0x20]++;}}}	//	выполнение функции I2C_func_WRITE_BYTE I2C_byte_SUM раз, с передачей байтов из массива BUS_buff_DATA
void	PCA9685::I2C_func_READ_WORD		(uint8_t I2C_byte_SUM)					{int I2C_var_I=0;	 BUS_buff_DATA[0x20]=0; for(I2C_var_I=0; I2C_var_I<I2C_byte_SUM; I2C_var_I++){BUS_buff_DATA[I2C_var_I]=I2C_func_READ_BYTE(I2C_var_I==I2C_byte_SUM-1?0:1);if(I2C_mass_STATUS[2]){BUS_buff_DATA[0x20]++;}}}	//	выполнение функции I2C_func_READ_BYTE  I2C_byte_SUM раз, при этом последний раз с отправкой NACK, а все предыдущие с отправкой ACK. Результат записываем в массив BUS_buff_DATA

