 
 
       loopTick = get_ticks();
        // *********** FSM ********************** // 
        switch (state){
            case T_SCHEDULE: 
                
                if(CHECK_TASK(blinkTask, loopTick)){ state = T_BLINK;}
                else if(CHECK_TASK(llCtrlTask, loopTick)){ state = T_LL_CTRL;}
                else if(CHECK_TASK(comsTask, loopTick)){ state = T_COMS;}
                else{state= T_SCHEDULE;}
                
                break;
            case T_POST: 
                // Test Serial 
                serialSend(&ser1, (uint8_t *)"Hello World\n", 13); 
                // Test IMU

                //mpu6050 = mpu6050Init(I2C_PORT, I2C_SCL, I2C_SDA);
                if(mpu6050.initalized == false){
                    comsSendMsg(&ser1, PUB_ERROR, "FAIL, mpu6050Init %d\n", mpu6050.data);
	            }
                state = T_SCHEDULE;
                break;
            case T_CALIB:
                state = T_SCHEDULE;
                break;
            case T_BLINK: 
                gpio_toggle(led.port, led.pin); 
                serialSend(&ser1, (uint8_t *)"Hello MARS\n", 12);   
                blinkTask.lastTick = loopTick;
                state = T_SCHEDULE;
                break; 
            case T_COMS: 
                serialSend(&ser1, (uint8_t *)"Hello PLUTO\n", 13); 
                comsTask.lastTick = loopTick;
                state = T_SCHEDULE;
                break; 
            case T_LL_CTRL: 
                mpu6050Read(&mpu6050);
                filterComplementary(&compFilt, &imu, &mpu6050.accel, &mpu6050.gyro);
                llCtrlTask.lastTick = loopTick;
                state = T_SCHEDULE;
                break;
            default:
                serialSend(&ser1, (uint8_t *)"Hello SUN\n", 11); 
                state= T_SCHEDULE;
                break;
        }