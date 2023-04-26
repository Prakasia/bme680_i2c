
/************		SAMPLE CODE FOR RPI PICO I2C0		***********
 * 					-----------------------------
 * **********		  BME680 TEMPERATURE SENSOR		***********
 * 
 * Editor : Shalu Prakasia
 * 
 * Connect the sensor to the GROVE3(I2C0)
 *
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>

#include <bme680_reg.h>

#define CONFIG_VALUE        (2<<2)         //FILTER COEFFICIENT = 15
#define TEMP_ENABLE         ((2<<5)|(1<<0))   //ENABLE TEMP BITS AND FORCED MODE
#define HUM_OVERSAMPLE      2               // HUMIDITY OVERSAMPLE 2

#define BME680_ADDR     0x77           //ADDRESS OF THE SENSOR
#define MY_STACK_SIZE   5000

#define I2C_NODE    DT_NODELABEL(i2c0) 	

void main(void)
{
    int ret;
    uint8_t buffer[20];

    int32_t var1, var2_1, var2_2, var2, var3, var4, var5, var6;
    
    int32_t par_t1, par_t2, par_t3;
    int32_t t_fine, temp_comp, adc_temp;

    uint16_t par_h1, par_h2;
    int8_t par_h3, par_h4, par_h5, par_h7;
    uint8_t par_h6;
    uint16_t hum_adc;
    int32_t hum_comp;

    const struct device *const dev = DEVICE_DT_GET(I2C_NODE);

/*******************   DEVICE READY CHECK   **************************/
    if (!device_is_ready(dev)) {
		printk("Bus %s not ready\n", dev->name);
		return;
	}
    else{
        printk("\n************************************************\n");
        printk("\nBus %s is Ready!!!",dev->name);
    }
/*********************************************************************/

/*******************   INITIALIZATION   ******************************/    

    // 1. Configure the sensor register 0x75 with filter coefficient
    ret = i2c_reg_write_byte(dev, BME680_ADDR, BME680_CONFIG, CONFIG_VALUE);
    if(ret<0){
        printk("\nError!! ");
        return;
    }
    else{
        printk("\nSuccessfully configured with filter coefficent");
    }

    // 2. Configure the register 0x74(Temperature oversampling) to measure temperature
    ret = i2c_reg_write_byte(dev, BME680_ADDR, BME680_CTRL_MEAS, TEMP_ENABLE);
    if(ret<0){
        printk("\nError!! ");
        // return;
    }
    else{
        printk("\nSuccessfully Enabled Temprature measurement with over sampling");
    }
    // 3. Configure the register 0x75(Humidity oversampling)
    ret = i2c_reg_write_byte(dev, BME680_ADDR, BME680_CTRL_HUM, HUM_OVERSAMPLE);
    if(ret<0){
        printk("\nError!! ");
        // return;
    }
    else{
        printk("\nSuccessfully Enabled Humidity measurement with over sampling 2");
        printk("\n****************************************************************\n");
    }

/*******************************************************************************************************/


/******* CALIBRATION PARAMETERS ********************/ 
    ret = i2c_burst_read(dev, BME680_ADDR, BME680_PAR, buffer, 10);
    if(ret<0){
    //    printf("\n Error reading calibration parameter");
    }
    else{
        par_h1 = (uint16_t)(((uint16_t)buffer[2] << 4) | (buffer[1] & 0x0F));
        par_h2 = (uint16_t)(((uint16_t)buffer[0] << 4) | (buffer[1] >> 4));
        par_h3 = (int8_t)buffer[3];
        par_h4 = (int8_t)buffer[4];
        par_h5 = (int8_t)buffer[5];
        par_h6 = (uint8_t)buffer[6];
        par_h7 = (int8_t)buffer[7];

        // printk("\nCALIBRATION PARAMETERS\nPAR1LSB = %d \t PAR1MSB = %d ", buffer[8], buffer[9]);
        par_t1 = (int32_t)(((uint16_t)buffer[9]) << 8) | (uint16_t)buffer[8]; 
    } 
    ret = i2c_burst_read(dev, BME680_ADDR, BME680_TEMP_PAR2, (buffer+10), 3);
    if(ret<0){
    //    printf("\n Error reading calibration parameter");
    }
    else{
        // printk("\nPAR2LSB = %d \t PAR2MSB = %d ", buffer[10], buffer[11]);
        par_t2 = (int32_t)(((int16_t)buffer[11]) << 8) | (int16_t)buffer[10];
        // printk("\nPAR3 %d", buffer[8]);
        par_t3 = (int32_t)buffer[12];
    }
    // printk("\npar_t1 = %d", par_t1);
    // printk("\npar_t2 = %d", par_t2);
    // printk("\npar_t3 = %d", par_t3);

    // printk("\npar_h1 = %d", par_h1);
    // printk("\npar_h2 = %d", par_h2);
    // printk("\npar_h3 = %d", par_h3);
    // printk("\npar_h4 = %d", par_h4);
    // printk("\npar_h5 = %d", par_h5);
    // printk("\npar_h6 = %d", par_h6);
    // printk("\npar_h7 = %d", par_h7);

/*******************************************************************************************************/

    while (1)
    {
        k_sleep(K_MSEC(3000));

/*******                READ NEW DATA BIT               *********************/
        ret = i2c_reg_read_byte(dev, BME680_ADDR, BME680_EAS_STATUS_0, (buffer+13));
        if(buffer[13] & 0x80){
            // printk("\nNew Data Available");

/********           READ TEMP (XLSB LSB MSB) AND HUMIDITY (MSB AND LSB)    *******************/ 
            ret = i2c_burst_read(dev, BME680_ADDR, BME680_TEMP_MSB, (buffer+14), 5);
            if(ret<0){
                // printf("\n Error reading temperature data");
            }
            else{
                // printk("\nXLSB = %d \t LSB = %d \t MSB = %d", buffer[16], buffer[15], buffer[14]);
                // printk("\nHUM_MSB = %d \tHUM_LSB = %d",buffer[17], buffer[18]);
            }
/*******               (xlsb (7 to 4 bits hence right shift))    ***************/ 
            adc_temp = (((int32_t)buffer[16])>>4) | (((int32_t)buffer[15])<<4) | (((int32_t)buffer[14])<<12);
            // printk("\nTEMPERATURE ADC RAW VALUE IS %d", adc_temp);
            hum_adc = (((uint16_t)buffer[17] << 8) | (uint16_t)buffer[18]);
            printk("\nHUMIDITY_ADC RAW VALUE IS %d", hum_adc);

/*******************   TEMPERATURE CALCULATION   ******************************/  
            var1 = ((int32_t)adc_temp>>3) - ((int32_t)par_t1 << 1);
            var2 = (var1 * (int32_t)par_t2) >> 11;
            var3 = (int32_t)(((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)par_t3 << 4)) >> 14);
            t_fine = (int32_t)(var2 + var3);
            temp_comp = (int32_t)(((t_fine * 5) + 128 ) >> 8);
/*******************************************************************************************************/

/*******************   HUMIDITY CALCULATION   ******************************/  
            var1 = (int32_t)hum_adc - (int32_t)((int32_t)par_h1 << 4) - ((temp_comp * (int32_t)par_h3) / ((int32_t)100) >> 1);
            var2_1 = (int32_t)par_h2;
            var2_2 = ((temp_comp * (int32_t)par_h4) / (int32_t)100) + (((temp_comp * ((temp_comp * (int32_t)par_h5) / ((int32_t)100))) >> 6) / ((int32_t)100)) +  (int32_t)(1 << 14);
            var2 = (var2_1 * var2_2) >> 10;
            var3 = var1 * var2;
            var4 = (((int32_t)par_h6 << 7) + ((temp_comp * (int32_t)par_h7) / ((int32_t)100))) >> 4;
            var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
            var6 = (var4 * var5) >> 1;
            hum_comp = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;
            
/*******************************************************************************************************/
            printk("\nTemperature :: %d.%06d degC", (temp_comp / 100), ((temp_comp%100)*1000));
            printk("\nHumidity :: %d.%06d",(hum_comp / 1000),((hum_comp % 1000)*1000));
        }

        else{
            printk("\nNo new data at the moment!! %x", buffer[0]);
        }
/*******            TRIGGER NEXT MEASUREMENT              ********************/         
        ret = i2c_reg_write_byte(dev, BME680_ADDR, BME680_CTRL_MEAS, TEMP_ENABLE);
        if(ret<0){
            printk("\nError!! ");
            return;
        }
    }
}
