
#include "i2c.h"

// I2C_1
void I2C_1_Start (void)
{
/**** STEPS FOLLOWED  ************
1. Send the START condition
2. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
*/

	I2C1->CR1 |= (1<<10);  // Enable the ACK
	I2C1->CR1 |= (1<<8);  // Generate START
	while (!(I2C1->SR1 & (1<<0)));  // Wait fror SB bit to set
}

void I2C_1_Write (uint8_t data)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Send the DATA to the DR Register
3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/
	while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
	I2C1->DR = data;
	while (!(I2C1->SR1 & (1<<2)));  // wait for BTF bit to set
}

void I2C_1_Address (uint8_t Address)
{
/**** STEPS FOLLOWED  ************
1. Send the Slave Address to the DR Register
2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of address transmission
3. clear the ADDR by reading the SR1 and SR2
*/
	I2C1->DR = Address;  //  send the address
	while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
	uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
}

void I2C_1_Stop (void)
{
	I2C1->CR1 |= (1<<9);  // Stop I2C
}

void I2C_1_WriteMulti (uint8_t *data, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Keep Sending DATA to the DR Register after performing the check if the TXE bit is set
3. Once the DATA transfer is complete, Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/
	while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
	while (size)
	{
		while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
		I2C1->DR = (uint32_t )*data++;  // send data
		size--;
	}

	while (!(I2C1->SR1 & (1<<2)));  // wait for BTF to set
}

void I2C_1_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. If only 1 BYTE needs to be Read
	a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
	c) Wait for the RXNE (Receive Buffer not Empty) bit to set
	d) Read the data from the DR

2. If Multiple BYTES needs to be read
  a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) Clear the ADDR bit by reading the SR1 and SR2 Registers
	c) Wait for the RXNE (Receive buffer not empty) bit to set
	d) Read the data from the DR
	e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
	f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the
		 second last data byte (after second last RxNE event)
	g) In order to generate the Stop/Restart condition, software must set the STOP/START bit
	   after reading the second last data byte (after the second last RxNE event)
*/

	int remaining = size;

/**** STEP 1 ****/
	if (size == 1)
	{
		/**** STEP 1-a ****/
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set

		/**** STEP 1-b ****/
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
		I2C1->CR1 |= (1<<9);  // Stop I2C

		/**** STEP 1-c ****/
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set

		/**** STEP 1-d ****/
		buffer[size-remaining] = I2C1->DR;  // Read the data from the DATA REGISTER

	}

/**** STEP 2 ****/
	else
	{
		/**** STEP 2-a ****/
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set

		/**** STEP 2-b ****/
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit

		while (remaining>2)
		{
			/**** STEP 2-c ****/
			while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set

			/**** STEP 2-d ****/
			buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer

			/**** STEP 2-e ****/
			I2C1->CR1 |= 1<<10;  // Set the ACK bit to Acknowledge the data received

			remaining--;
		}

		// Read the SECOND LAST BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;

		/**** STEP 2-f ****/
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit

		/**** STEP 2-g ****/
		I2C1->CR1 |= (1<<9);  // Stop I2C

		remaining--;

		// Read the Last BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer
	}

}








// I2C_2
void I2C_2_Start (void)
{
/**** STEPS FOLLOWED  ************
1. Send the START condition 
2. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
*/	

	I2C2->CR1 |= (1<<10);  // Enable the ACK
	I2C2->CR1 |= (1<<8);  // Generate START
	while (!(I2C2->SR1 & (1<<0)));  // Wait fror SB bit to set
}

void I2C_2_Write (uint8_t data)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Send the DATA to the DR Register
3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/	
	while (!(I2C2->SR1 & (1<<7)));  // wait for TXE bit to set
	I2C2->DR = data;
	while (!(I2C2->SR1 & (1<<2)));  // wait for BTF bit to set
}

void I2C_2_Address (uint8_t Address)
{
/**** STEPS FOLLOWED  ************
1. Send the Slave Address to the DR Register
2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of address transmission
3. clear the ADDR by reading the SR1 and SR2
*/	
	I2C2->DR = Address;  //  send the address
	while (!(I2C2->SR1 & (1<<1)));  // wait for ADDR bit to set
	uint8_t temp = I2C2->SR1 | I2C2->SR2;  // read SR1 and SR2 to clear the ADDR bit
}
	
void I2C_2_Stop (void)
{
	I2C2->CR1 |= (1<<9);  // Stop I2C
}

void I2C_2_WriteMulti (uint8_t *data, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
2. Keep Sending DATA to the DR Register after performing the check if the TXE bit is set
3. Once the DATA transfer is complete, Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
*/	
	while (!(I2C2->SR1 & (1<<7)));  // wait for TXE bit to set
	while (size)
	{
		while (!(I2C2->SR1 & (1<<7)));  // wait for TXE bit to set
		I2C2->DR = (uint32_t )*data++;  // send data
		size--;
	}
	
	while (!(I2C2->SR1 & (1<<2)));  // wait for BTF to set
}

void I2C_2_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{
/**** STEPS FOLLOWED  ************
1. If only 1 BYTE needs to be Read
	a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
	c) Wait for the RXNE (Receive Buffer not Empty) bit to set
	d) Read the data from the DR

2. If Multiple BYTES needs to be read
  a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
	b) Clear the ADDR bit by reading the SR1 and SR2 Registers
	c) Wait for the RXNE (Receive buffer not empty) bit to set
	d) Read the data from the DR 
	e) Generate the Acknowlegment by settint the ACK (bit 10 in SR1)
	f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the 
		 second last data byte (after second last RxNE event)
	g) In order to generate the Stop/Restart condition, software must set the STOP/START bit 
	   after reading the second last data byte (after the second last RxNE event)
*/		
	
	int remaining = size;
	
/**** STEP 1 ****/	
	if (size == 1)
	{
		/**** STEP 1-a ****/	
		I2C2->DR = Address;  //  send the address
		while (!(I2C2->SR1 & (1<<1)));  // wait for ADDR bit to set
		
		/**** STEP 1-b ****/	
		I2C2->CR1 &= ~(1<<10);  // clear the ACK bit
		uint8_t temp = I2C2->SR1 | I2C2->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
		I2C2->CR1 |= (1<<9);  // Stop I2C

		/**** STEP 1-c ****/	
		while (!(I2C2->SR1 & (1<<6)));  // wait for RxNE to set
		
		/**** STEP 1-d ****/	
		buffer[size-remaining] = I2C2->DR;  // Read the data from the DATA REGISTER
		
	}

/**** STEP 2 ****/		
	else 
	{
		/**** STEP 2-a ****/
		I2C2->DR = Address;  //  send the address
		while (!(I2C2->SR1 & (1<<1)));  // wait for ADDR bit to set
		
		/**** STEP 2-b ****/
		uint8_t temp = I2C2->SR1 | I2C2->SR2;  // read SR1 and SR2 to clear the ADDR bit
		
		while (remaining>2)
		{
			/**** STEP 2-c ****/
			while (!(I2C2->SR1 & (1<<6)));  // wait for RxNE to set
			
			/**** STEP 2-d ****/
			buffer[size-remaining] = I2C2->DR;  // copy the data into the buffer
			
			/**** STEP 2-e ****/
			I2C2->CR1 |= 1<<10;  // Set the ACK bit to Acknowledge the data received
			
			remaining--;
		}
		
		// Read the SECOND LAST BYTE
		while (!(I2C2->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C2->DR;
		
		/**** STEP 2-f ****/
		I2C2->CR1 &= ~(1<<10);  // clear the ACK bit
		
		/**** STEP 2-g ****/
		I2C2->CR1 |= (1<<9);  // Stop I2C
		
		remaining--;
		
		// Read the Last BYTE
		while (!(I2C2->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C2->DR;  // copy the data into the buffer
	}	
	
}
