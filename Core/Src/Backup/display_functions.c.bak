

#include "display_functions.h"
#include "DISP_14_SEG.h"


char display_buffer[] = "        "; // 8 caractere
char mess_frequency[30];
char mess_freq_static[30];
char mess_volume[30];
extern uint16_t displaybuffer[8];

void clear_display(void)
{
  for (uint8_t i=0; i<8; i++)
  {
    displaybuffer[i] = 0;
  }
}
void disp_vol(uint32_t vol)  // 0-60
{
    uint8_t vol_0 = vol / 10;  // prima cifra
    uint8_t vol_1 = vol % 10;  // a doua cifra
    if(vol_0 > 0)
    {
    	writeDigitAscii(0, 'V', false);
    	writeDigitAscii(1, 'O', false);
    	writeDigitAscii(2, 'L', false);
    	writeDigitAscii(3, ' ', false);
    	writeDigitAscii(4, vol_0 + 48, false);
    	writeDigitAscii(5, vol_1 + 48, false);
    	writeDisplay(DISPLAY_ADDRESS);
    }
    else
    {
    	writeDigitAscii(0, 'V', false);
    	writeDigitAscii(1, 'O', false);
    	writeDigitAscii(2, 'L', false);
    	writeDigitAscii(3, ' ', false);
    	writeDigitAscii(4, ' ', false);
    	writeDigitAscii(5, vol_1 + 48, false);
    	writeDisplay(DISPLAY_ADDRESS);
    }


}
void disp_freq(uint32_t freq)
{
	if (freq >= 10000)
	{
		uint16_t freq_0 = freq / 10000; // prima cifra
		uint16_t f_0 = freq % 10000;
		uint16_t freq_1 = f_0 / 1000;   // a doua cifra
		uint16_t f_1 =  f_0 % 1000;
		uint16_t freq_2 = f_1 / 100;   // a treia cifra  (*cu punct jos)
		uint16_t freq_3 = f_1 %  100;
		uint16_t freq_4 = freq_3 / 10;   // a patra cifra

		writeDigitAscii(0, freq_0 + 48, false);
		writeDigitAscii(1, freq_1 + 48, false);
		writeDigitAscii(2, freq_2 + 48, true);
		writeDigitAscii(3, freq_4 + 48, false);
		writeDigitAscii(4, ' ', false);
		writeDigitAscii(5, 'M', false);
		writeDigitAscii(6, 'H', false);
		writeDigitAscii(7, 'Z', false);
		writeDisplay(DISPLAY_ADDRESS);
	}
	else
	{
		uint16_t freq_0 = freq / 1000; // prima cifra
		uint16_t f_0 = freq % 1000;
		uint16_t freq_1 = f_0 / 100;   // a doua cifra
		uint16_t f_1 =  f_0 % 100;
		uint16_t freq_2 = f_1 / 10;   // a treia cifra  (*cu punct jos)
		//uint16_t freq_3 = f_1 % 100;   // a patra cifra

		writeDigitAscii(0, ' ', false);
		writeDigitAscii(1, freq_0 + 48, false);
		writeDigitAscii(2, freq_1 + 48, true);
		writeDigitAscii(3, freq_2 + 48, false);
		writeDigitAscii(4, ' ', false);
		writeDigitAscii(5, 'M', false);
		writeDigitAscii(6, 'H', false);
		writeDigitAscii(7, 'Z', false);
		writeDisplay(DISPLAY_ADDRESS);
	}
}
void display_static_message(char *mess)
{
 // uint8_t string_length = strlen(mess);

//  if(string_length <= 8)
  {
	writeDigitAscii(0, mess[0], false);
	writeDigitAscii(1, mess[1], false);
	writeDigitAscii(2, mess[2], false);
	writeDigitAscii(3, mess[3], false);
	writeDigitAscii(4, mess[4], false);
	writeDigitAscii(5, mess[5], false);
	writeDigitAscii(6, mess[6], false);
	writeDigitAscii(7, mess[7], false);
	writeDisplay(DISPLAY_ADDRESS);
  }
//  else if((string_length > 9)&&(string_length < 17))  //
//  {
//
//
//  }
//  else if(string_length > )
//  {
//
//  }

}
void show_on_display(char *message, uint8_t index_of_point)
{
	if(index_of_point < 9)
	{
		switch (index_of_point)
		{
			case 0:
				writeDigitAscii(0, message[0], true);
				writeDigitAscii(1, message[1], false);
				writeDigitAscii(2, message[2], false);
				writeDigitAscii(3, message[3], false);
				writeDigitAscii(4, message[4], false);
				writeDigitAscii(5, message[5], false);
				writeDigitAscii(6, message[6], false);
				writeDigitAscii(7, message[7], false);
				writeDisplay(DISPLAY_ADDRESS);
				break;
			case 1:
				writeDigitAscii(0, message[0], false);
				writeDigitAscii(1, message[1], true);
				writeDigitAscii(2, message[2], false);
				writeDigitAscii(3, message[3], false);
				writeDigitAscii(4, message[4], false);
				writeDigitAscii(5, message[5], false);
				writeDigitAscii(6, message[6], false);
				writeDigitAscii(7, message[7], false);
				writeDisplay(DISPLAY_ADDRESS);
				break;
			case 2:
				writeDigitAscii(0, message[0], false);
				writeDigitAscii(1, message[1], false);
				writeDigitAscii(2, message[2], true);
				writeDigitAscii(3, message[3], false);
				writeDigitAscii(4, message[4], false);
				writeDigitAscii(5, message[5], false);
				writeDigitAscii(6, message[6], false);
				writeDigitAscii(7, message[7], false);
				writeDisplay(DISPLAY_ADDRESS);
				break;
			case 3:
				writeDigitAscii(0, message[0], false);
				writeDigitAscii(1, message[1], false);
				writeDigitAscii(2, message[2], false);
				writeDigitAscii(3, message[3], true);
				writeDigitAscii(4, message[4], false);
				writeDigitAscii(5, message[5], false);
				writeDigitAscii(6, message[6], false);
				writeDigitAscii(7, message[7], false);
				writeDisplay(DISPLAY_ADDRESS);
				break;
			case 4:
				writeDigitAscii(0, message[0], false);
				writeDigitAscii(1, message[1], false);
				writeDigitAscii(2, message[2], false);
				writeDigitAscii(3, message[3], false);
				writeDigitAscii(4, message[4], true);
				writeDigitAscii(5, message[5], false);
				writeDigitAscii(6, message[6], false);
				writeDigitAscii(7, message[7], false);
				writeDisplay(DISPLAY_ADDRESS);
				break;
			case 5:
				writeDigitAscii(0, message[0], false);
				writeDigitAscii(1, message[1], false);
				writeDigitAscii(2, message[2], false);
				writeDigitAscii(3, message[3], false);
				writeDigitAscii(4, message[4], false);
				writeDigitAscii(5, message[5], true);
				writeDigitAscii(6, message[6], false);
				writeDigitAscii(7, message[7], false);
				writeDisplay(DISPLAY_ADDRESS);
				break;
			case 6:
				writeDigitAscii(0, message[0], false);
				writeDigitAscii(1, message[1], false);
				writeDigitAscii(2, message[2], false);
				writeDigitAscii(3, message[3], false);
				writeDigitAscii(4, message[4], false);
				writeDigitAscii(5, message[5], false);
				writeDigitAscii(6, message[6], true);
				writeDigitAscii(7, message[7], false);
				writeDisplay(DISPLAY_ADDRESS);
				break;
			case 7:
				writeDigitAscii(0, message[0], false);
				writeDigitAscii(1, message[1], false);
				writeDigitAscii(2, message[2], false);
				writeDigitAscii(3, message[3], false);
				writeDigitAscii(4, message[4], false);
				writeDigitAscii(5, message[5], false);
				writeDigitAscii(6, message[6], false);
				writeDigitAscii(7, message[7], true);
				writeDisplay(DISPLAY_ADDRESS);
				break;
			default:
				writeDigitAscii(0, message[0], false);
				writeDigitAscii(1, message[1], false);
				writeDigitAscii(2, message[2], false);
				writeDigitAscii(3, message[3], false);
				writeDigitAscii(4, message[4], false);
				writeDigitAscii(5, message[5], false);
				writeDigitAscii(6, message[6], false);
				writeDigitAscii(7, message[7], false);
				writeDisplay(DISPLAY_ADDRESS);
				break;
		}
	}
	else
	{
		writeDigitAscii(0, message[0], false);
		writeDigitAscii(1, message[1], false);
		writeDigitAscii(2, message[2], false);
		writeDigitAscii(3, message[3], false);
		writeDigitAscii(4, message[4], false);
		writeDigitAscii(5, message[5], false);
		writeDigitAscii(6, message[6], false);
		writeDigitAscii(7, message[7], false);
		writeDisplay(DISPLAY_ADDRESS);
	}
}
uint8_t find_decimal_point_in_string(char *message)  // TODO de verificat
{
	// 15 means no point to display
	uint8_t string_length = strlen(message);
	uint8_t count_digits = 0;
	uint8_t count_not_digits = 0;
    int8_t index_last_digit  = -1;
    for (uint8_t k = 0; k <= string_length; k++)
    {
        if (isdigit(message[k]) == 0) // nu e cifra
        {
            count_not_digits++;
        }
        else
        {
            count_digits++;         // cate cifre apar pe display la un moment dat
            index_last_digit = k;
        }
    }

    if (count_digits <= 1)
    {
        return 15;
    }

    else if (count_digits == 2)
    {
        if (index_last_digit > 6)
        {
          if((message[index_last_digit - 2] == ' ') && (message[index_last_digit - 1] != '1'))
          {
        	 return 7;                            /// test ok
          }

         if(message[index_last_digit - 1] == '1')
             return 15;
         else
             return 11;
        }

        else
         return 0;
    }

    else if (count_digits == 3)
    {
    	if(message[index_last_digit - 2] == '0')
    	{
    		return index_last_digit - 1; /// nu
    	}
    	if(index_last_digit > 6)
    	{
    		if(message[index_last_digit - 1] == '0')
    			return index_last_digit;
    		else
    			return index_last_digit - 1; //nu

    	}
    	else
            return index_last_digit - 1; ///// nu
    }

    else if (count_digits == 4)
    {
        return index_last_digit - 1;
    }

    else
    {
        return index_last_digit;
    }
}
void display_scrolling_message(char *mess)
{
	uint8_t string_length = strlen(mess);
	static uint32_t current_count = SCROLL_SPEED;
	current_count++;


	if(string_length > NR_OF_DIGITS)
	{
		strncpy(display_buffer, mess, NR_OF_DIGITS);   // copies first 8 characters from the string 'mess' to 'display_buffer'
		if(current_count > SCROLL_SPEED)
		{
			uint8_t index_of_point = find_decimal_point_in_string(display_buffer);
			show_on_display(display_buffer, index_of_point);
			left_rotate(mess, ONE_STEP, string_length);
			current_count = 0;
		}
	}
	else
	{
		static bool first_run = true;
		if (first_run)
		{
			strncpy(display_buffer, mess, string_length);        // copies characters from the string 'mess' to 'display_buffer'
			//left_rotate(display_buffer, 5, string_length);      // start message from right side
			first_run = false;
		}
		if(current_count > SCROLL_SPEED)
		{
			uint8_t index_of_point = find_decimal_point_in_string(display_buffer);
			show_on_display(display_buffer, index_of_point);
			left_rotate(display_buffer, ONE_STEP, NR_OF_DIGITS);           // start message from right side
			current_count = 0;
		}
	}
}

void left_rotate_by_one(char arr[], int n)
{
    int j, temp;
    temp = arr[0];
    for (j = 0; j < n - 1; j++)
        arr[j] = arr[j + 1];
    arr[j] = temp;
}

// Function to left rotate arr[] of size n by d
void left_rotate(char arr[], int d, int n)
{
    int k;
    for (k = 0; k < d; k++)
        left_rotate_by_one(arr, n);
}

void populate_freq_array(uint32_t freq)
{
	// atentie modificarea mess_frequency poate duce la refacerea functiei
	//   uint8_t find_decimal_point_in_string()


	uint16_t freq_0 = freq / 10000; // prima cifra
	uint16_t f_0 = freq % 10000;
	uint16_t freq_1 = f_0 / 1000;   // a doua cifra
	uint16_t f_1 =  f_0 % 1000;
	uint16_t freq_2 = f_1 / 100;   // a treia cifra  (*cu punct jos)
	uint16_t f_2 =  f_1 % 100;
	uint16_t freq_3 = f_2 / 10;   // a patra cifra

	mess_frequency[0] = 'F';
	mess_frequency[1] = 'R';
	mess_frequency[2] = 'E';
	mess_frequency[3] = 'Q';
	mess_frequency[4] = 'U';
	mess_frequency[5] = 'E';
	mess_frequency[6] = 'N';
	mess_frequency[7] = 'C';
	mess_frequency[8] = 'Y';
	mess_frequency[9] = ' ';

	//message_frequency[5] = freq_0 + 48;
	if(freq_0 + 48 == '0')
	{
		mess_frequency[10] = ' ';
	}
	else
	{
		mess_frequency[10] = freq_0 + 48;
	}
    mess_frequency[11] = freq_1 + 48;
    mess_frequency[12] = freq_2 + 48; //
	mess_frequency[13] = freq_3 + 48; //
	mess_frequency[14] = ' ';
	mess_frequency[15] = 'M';
	mess_frequency[16] = 'H';
	mess_frequency[17] = 'Z';
	mess_frequency[18] = ' ';
	mess_frequency[19] = ' ';
	mess_frequency[20] = '\0';




	mess_freq_static[0] = 'F';
	mess_freq_static[1] = ' ';
	//message_freq_static[2] = freq_0 + 48;
	if(freq_0 + 48 == '0')
	{
		mess_freq_static[2] = ' ';
	}
	else
	{
		mess_freq_static[2] = freq_0 + 48;
	}
	mess_freq_static[3] = freq_1 + 48;
	mess_freq_static[4] = freq_2 + 48; // punct;
	//mess_freq_static[5] = freq_3 + 48;
	mess_freq_static[6] = ' ';
	mess_freq_static[7] = 'M';
	mess_freq_static[8] = 'H';
	mess_freq_static[9] = 'Z';
	mess_freq_static[10] = ' ';
	mess_freq_static[11] = ' ';

}

void populate_vol_array(uint16_t vol)
{
  // char message_volume[] = "VOLUME 10";
	uint8_t v1 = vol / 10;
	uint8_t v2 = vol % 10;

	mess_volume[0] = 'V';
	mess_volume[1] = 'O';
	mess_volume[2] = 'L';
	mess_volume[3] = 'U';
	mess_volume[4] = 'M';
	mess_volume[5] = 'E';
	mess_volume[6] = ' ';
	//message_volume[7] = v1 + 48;
	if(v1 + 48 == '0')
	{
		mess_volume[7] = ' ';
	}
	else
	{
		mess_volume[7] = v1 + 48;
	}
	mess_volume[8] = v2 + 48;
}


uint8_t find_3_space_in_string(char *message)
{

    int i=0;
    while (message[i]!='\0')
    {
        if (message[i]==' ')
        {
            if (message[i+1]==' ')
            {
               if (message[i+2]==' ')
               {
                  break;
               }
            }
        }
        i++;
    }
    return i + 2;
}
//void disp_static(char *message) // TODO
//{
//	uint8_t string_length = strlen(message);
//
//	if(string_length > NR_OF_DIGITS)
//	{
//
//	}
//	else
//	{
//		strncpy(display_buffer, message, string_length);   // copies first 12 characters from the string 'mess' to 'display_buffer'
//		int8_t index_of_point = find_decimal_point_in_string(display_buffer);
//		show_on_display(display_buffer, index_of_point);
//	}
//}
