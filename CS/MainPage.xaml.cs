// Copyright (c) Microsoft. All rights reserved.

using System;
using System.Text;
using System.Threading;
using Windows.UI.Xaml.Controls;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;
using System.Threading.Tasks;
using Microsoft.Azure.Devices.Client;
using Newtonsoft.Json;
using System.Diagnostics;


namespace I2CAccelerometer
{

    enum ADS1115Gain:ushort
    {
        g2_3 = 0x0000,
        g1   = 0x0200,
        g2   = 0x0400,
        g4  = 0x0600,
        g8  = 0x0800,
        g16 = 0x0A00   

    }

    struct TPData
    {
        public double temperature;
        public double pressure;
    };


    /*! 
     * @brief This structure holds all device specific calibration parameters 
     */
    struct BMPCalibParam
    {
        public short ac1;      /**<calibration ac1 data*/
        public short ac2;      /**<calibration ac2 data*/
        public short ac3;      /**<calibration ac3 data*/
        public ushort ac4;     /**<calibration ac4 data*/
        public ushort ac5;     /**<calibration ac5 data*/
        public ushort ac6;     /**<calibration ac6 data*/
        public short b1;       /**<calibration b1 data*/
        public short b2;       /**<calibration b2 data*/
        public short mb;       /**<calibration mb data*/
        public short mc;       /**<calibration mc data*/
        public short md;       /**<calibration md data*/
    };

    /*! 
          * @brief This structure holds BMP180 initialization parameters 
          */
    struct BMPInits
    {
        public BMPCalibParam calibParam;   /**<calibration data*/
        public byte mode;                          /**<power mode*/
        public byte chip_id;                       /**<chip id*/
        public byte ml_version;                    /**<ml version*/
        public byte al_version;                    /**<al version*/
        public byte dev_addr;                      /**<device address*/
        public byte sensortype;                    /**< sensor type*/
         
        public int param_b5;                       /**<pram*/
        public int number_of_samples;              /**<sample calculation*/
        public short oversamp_setting;             /**<oversampling setting*/
        public short sw_oversamp;                  /**<software oversampling*/
    };

    /// <summary>
    /// Sample app that reads data over I2C from an attached ADXL345 accelerometer
    /// </summary>
    public sealed partial class MainPage : Page
    {
        // private const byte ACCEL_I2C_ADDR = 0x53;           /* 7-bit I2C address of the ADXL345 with SDO pulled low */
        private const byte ACCEL_I2C_ADDR = 0x77;           /* 7-bit I2C address of the ADXL345 with SDO pulled low */
        private const byte ACCEL_REG_POWER_CONTROL = 0x2D;  /* Address of the Power Control register */
        private const byte ACCEL_REG_DATA_FORMAT = 0x31;    /* Address of the Data Format register   */
        private const byte ACCEL_REG_X = 0x32;              /* Address of the X Axis data register   */
        private const byte ACCEL_REG_Y = 0x34;              /* Address of the Y Axis data register   */
        private const byte ACCEL_REG_Z = 0x36;              /* Address of the Z Axis data register   */

        //private I2cDevice I2CAccel;
        private Timer periodicTimer;

        /*
        *   Constants for BMP180 Barometer I2C chip
        */
        private const byte BMP_I2C_ADDR = 0x77;           /* 7-bit I2C address of the BMP180*/
        private const byte ADS1x15_I2C_ADDR = 0x48;           /* 7-bit I2C address of the ADS1115*/
        // Register and other configuration values: 

        private const byte      ADS1x15_POINTER_CONVERSION = 0x00;
        private const byte      ADS1x15_POINTER_CONFIG = 0x01;
        private const byte      ADS1x15_POINTER_LOW_THRESHOLD = 0x02;
        private const byte      ADS1x15_POINTER_HIGH_THRESHOLD = 0x03;
        private const ushort    ADS1x15_CONFIG_OS_SINGLE = 0x8000;
        private const byte      ADS1x15_CONFIG_MUX_OFFSET = 12;


        /***************************************************************/
        /**\name	REGISTER ADDRESS DEFINITION       */
        /***************************************************************/
        /*register definitions */

        private const byte BMP_PROM_START__ADDR = 0xAA;
        private const byte BMP_PROM_DATA__LEN = 22;

        private const byte BMP_CHIP_ID_REG = 0xD0;
        private const byte BMP_VERSION_REG = 0xD1;
        private const byte BMP_SOFT_RESET_REG = 0xE0;


        private const byte BMP_CONTROL = 0xF4;
        private const byte BMP_READPRESSUREDATA = 0xF6;
        private const byte BMP_READTEMPDATA = 0xF6;
        private const byte BMP_READTEMPCMD = 0x2e;
        private const byte BMP_READPRESSURECMD = 0x34;

        /****************************************************/
        /**\name	ARRAY SIZE DEFINITIONS      */
        /***************************************************/
        private const byte BMP_TEMPERATURE_DATA_BYTES = 2;
        private const byte BMP_PRESSURE_DATA_BYTES = 3;
        private const byte BMP_TEMPERATURE_LSB_DATA = 1;
        private const byte BMP_TEMPERATURE_MSB_DATA = 0;
        private const byte BMP_PRESSURE_MSB_DATA = 0;
        private const byte BMP_PRESSURE_LSB_DATA = 1;
        private const byte BMP_PRESSURE_XLSB_DATA = 2;


        private const byte BMP_CALIB_DATA_SIZE = 2;
        private const byte BMP_CALIB_PARAM_AC1_MSB = 0;
        private const byte BMP_CALIB_PARAM_AC1_LSB = 1;
        private const byte BMP_CALIB_PARAM_AC2_MSB = 2;
        private const byte BMP_CALIB_PARAM_AC2_LSB = 3;
        private const byte BMP_CALIB_PARAM_AC3_MSB = 4;
        private const byte BMP_CALIB_PARAM_AC3_LSB = 5;
        private const byte BMP_CALIB_PARAM_AC4_MSB = 6;
        private const byte BMP_CALIB_PARAM_AC4_LSB = 7;
        private const byte BMP_CALIB_PARAM_AC5_MSB = 8;
        private const byte BMP_CALIB_PARAM_AC5_LSB = 9;
        private const byte BMP_CALIB_PARAM_AC6_MSB = 10;
        private const byte BMP_CALIB_PARAM_AC6_LSB = 11;
        private const byte BMP_CALIB_PARAM_B1_MSB = 12;
        private const byte BMP_CALIB_PARAM_B1_LSB = 13;
        private const byte BMP_CALIB_PARAM_B2_MSB = 14;
        private const byte BMP_CALIB_PARAM_B2_LSB = 15;
        private const byte BMP_CALIB_PARAM_MB_MSB = 16;
        private const byte BMP_CALIB_PARAM_MB_LSB = 17;
        private const byte BMP_CALIB_PARAM_MC_MSB = 18;
        private const byte BMP_CALIB_PARAM_MC_LSB = 19;
        private const byte BMP_CALIB_PARAM_MD_MSB = 20;
        private const byte BMP_CALIB_PARAM_MD_LSB = 21;



        private const int BMP_TEMP_CONVERSION_TIME = 5;

        private I2cDevice I2CBmp;
        private I2cDevice I2CAdc;
        private BMPInits BMPInitVals = new BMPInits();
        byte[] CmdBuf_ReadTmp = new byte[] { BMP_CONTROL, BMP_READTEMPCMD };
        byte[] CmdBuf_ReadData = new byte[] { BMP_READTEMPDATA };
        byte[] CmdBuf_ReadPreas = new byte[] { BMP_CONTROL, BMP_READPRESSURECMD };
        byte[] CmdBuf_ReadPreasData = new byte[] { BMP_READPRESSUREDATA };
        byte[] CmdBuf_ChipID = new byte[] { BMP_CHIP_ID_REG };               /* Read Chip ID reg */
        byte[] CmdBuf_PromData = new byte[] { BMP_PROM_START__ADDR };          /* Read Prom Data */


        byte[] ReadByte = new byte[1];
        byte[] ReadBuf = new byte[2];
        byte[] ReadPresBuf = new byte[3];
        byte[] ReadEEPromBuff = new byte[BMP_PROM_DATA__LEN];

        static private String mTemperature = "0.0";
        static private String mPresure = "0.0";
        public DeviceClient devClient;
        Timer barTimer;
        Timer adcTimer;


        public MainPage()
        {
            this.InitializeComponent();

            /* Register for the unloaded event so we can clean up upon exit */
            Unloaded += MainPage_Unloaded;
            SendDeviceToCloudMessagesAsync();

            /* Initialize the I2C bus, accelerometer, and timer */
            InitI2CBMP();
            AutoResetEvent autoEvent = new AutoResetEvent(false);
//            barTimer = new Timer(new TimerCallback(BarTimer_Tick), autoEvent, 1000, 60000);
            adcTimer = new Timer(new TimerCallback(adcTimer_Tick), autoEvent, 1000, 2000);

        }

        private async void adcTimer_Tick(object state)
        {
            ushort res = await ReadAdc(0, 0);
            //throw new NotImplementedException();
        }

        static async void SendDeviceToCloudMessagesAsync()
        {
            string iotHubUri = "Bellatrix.azure-devices.net"; // ! put in value !
            string deviceId  = "MyBarometer"; // ! put in value !
            string deviceKey = "elWugFxBcgv5h82WJYljwvXV14UQU+rJ4hLBXwyG3FY="; // ! put in value !

            var dc = DeviceClient.Create(iotHubUri,
                    AuthenticationMethodFactory.
                        CreateAuthenticationWithRegistrySymmetricKey(deviceId, deviceKey),
                    TransportType.Http1);

            var str = "Hello, Cloud!";
            var message = new Message(Encoding.ASCII.GetBytes(str));

            await dc.SendEventAsync(message);
            
        }

        private async void BarTimer_Tick(Object stateInfo)
        {
            // code goes here 
             await SendDeviceDataToCloudMessagesAsync();
        }

        static async Task SendDeviceDataToCloudMessagesAsync()
        {
            string iotHubUri = "Bellatrix.azure-devices.net"; // ! put in value !
            string deviceId  = "MyBarometer"; // ! put in value !
            string deviceKey = "dlWugFxBcgv5h82WJYljwvXV14UQU+rJ4hLBXwyG3FY="; // ! put in value !

            var deviceClient = DeviceClient.Create(iotHubUri,
                    AuthenticationMethodFactory.
                        CreateAuthenticationWithRegistrySymmetricKey(deviceId, deviceKey),
                    TransportType.Http1);
            var telemetryDataPoint = new
            {
                deviceId = deviceId,
                temperature = mTemperature,
                presure = mPresure,
                time = DateTime.Now

                //time = DateTime.Now.ToLocalTime()
            };
            
            
            var messageString = JsonConvert.SerializeObject(telemetryDataPoint);
            var message = new Message(Encoding.ASCII.GetBytes(messageString));

            await deviceClient.SendEventAsync(message);
           // Console.WriteLine("{0} > Sending message: {1}", DateTime.Now, messageString);
        }
        //
        /// <summary>
        /// Calculate Calibrated Temperature
        /// </summary>
        /// <param name="RawTemp"></param>
        /// <param name="calibParam"></param>
        /// <returns>deg C</returns>
        private double CalculateCalibratedTemperature(short RawTemp, BMPCalibParam calibParam)
        {
            short ac1 = calibParam.ac1;
            short ac2 = calibParam.ac2;
            short ac3 = calibParam.ac3;
            ushort ac4 = calibParam.ac4;
            ushort ac5 = calibParam.ac5;
            ushort ac6 = calibParam.ac6;
            short b1 = calibParam.b1;
            short b2 = calibParam.b2;
            short mb = calibParam.mb;
            short mc = calibParam.mc;
            short md = calibParam.md;

            int X1, X2, B5;
            X1 = ((RawTemp - ac6) * ac5) >> 15;
            X2 = (mc << 11) / (X1 + md);
            B5 = X1 + X2;
            return ((B5 + 8) >> 4)/10.0;
        }

        /// <summary>
        /// Calculate Calibrated Preasure
        /// </summary>
        /// <param name="rawPreasure"></param>
        /// <param name="calibParam"></param>
        /// <returns> kPa </returns>
        private double CalculateCalibratedPreasure(int UP, short UT,short oss, BMPCalibParam calibParam)
        {
            short ac1   = calibParam.ac1;
            short ac2   = calibParam.ac2;
            short ac3   = calibParam.ac3;
            ushort ac4  = calibParam.ac4;
            ushort ac5  = calibParam.ac5;
            ushort ac6  = calibParam.ac6;
            short b1    = calibParam.b1;
            short b2    = calibParam.b2;
            short mb    = calibParam.mb;
            short mc    = calibParam.mc;
            short md    = calibParam.md;
            int p;

            int X1, X2, X3, B3, B5, B6;
             
            uint B4, B7;

            X1 = ((UT - ac6) * ac5) >> 15;
            X2 = (mc << 11) / (X1 + md);
            B5 = X1 + X2;

            B6 = B5 - 4000;
            X1 = (b2 * ((B6 * B6) >> 12)) >> 11;
            X2 = (ac2 * B6) >> 11;
            X3 = X1 + X2;
            B3 = (((ac1 * 4 + X3) << oss) + 2) / 4;
            X1 = (ac3 * B6) >> 13;
            X2 = (b1 * (B6 * B6) >> 12) >> 16;
            X3 = ((X1 * X2) + 2) >> 2;
            B4 = (uint)(ac4 * ((X3 + 32768) ) >> 15);
            B7 = (uint)((UP - B3) * (50000 >> oss));
            if( B7 < 80000000 )
            {
                p = (int)((B7 * 2) / B4);
            }
            else
            {
                p = (int)(B7 / B4) * 2;
            }

            X1 = (p >> 8) ^ 2;
            X1 = (X1 * 3038) >> 16;
            X2 = (-7375 * p) >> 15;
            p +=  ((X1 + X2 + 3791) >> 4);

            return p;
        }

        private async void InitI2CBMP()
        {
            BMPInitVals.oversamp_setting = 0;
            try {
                string aqs = I2cDevice.GetDeviceSelector();                     /* Get a selector string that will return all I2C controllers on the system */
                var dis = await DeviceInformation.FindAllAsync(aqs);            /* Find the I2C bus controller device with our selector string           */
                if (dis.Count == 0)
                {
                    Text_Status.Text = "No I2C controllers were found on the system";
                    return;
                }


                var settings = new I2cConnectionSettings(BMP_I2C_ADDR);
                settings.BusSpeed = I2cBusSpeed.StandardMode;
                I2CBmp = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings */
                if (I2CBmp == null)
                {
                    Text_Status.Text = string.Format(
                        "Slave address {0} on I2C Controller {1} is currently in use by " +
                        "another application. Please ensure that no other applications are using I2C.",
                        settings.SlaveAddress,
                        dis[0].Id);
                    return;
                }

                settings = new I2cConnectionSettings(ADS1x15_I2C_ADDR);
                settings.BusSpeed = I2cBusSpeed.StandardMode;
                I2CAdc = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings */
                if (I2CAdc == null)
                {
                    Text_Status.Text = string.Format(
                        "Slave address {0} on I2C Controller {1} is currently in use by " +
                        "another application. Please ensure that no other applications are using I2C.",
                        settings.SlaveAddress,
                        dis[0].Id);
                    return;
                }

            }
            catch (Exception ex)
            {
                Text_Status.Text = "Failed to create the device: " + ex.Message;
                return;
            }
            /* 
                       * Initialize the accelerometer:
                       *
                       * For this device, we create 2-byte write buffers:
                       * The first byte is the register address we want to write to.
                       * The second byte is the contents that we want to write to the register. 
                       */


            try
            {   // Lets try and read the EEprom Callibration data out.

                I2cTransferResult tResult = I2CBmp.WriteReadPartial(CmdBuf_ChipID, ReadByte);
                if (tResult.Status == I2cTransferStatus.FullTransfer && ReadByte[0] == 0x55)
                {   // The I2C transfer succeded and we recieved the correct Chip ID from the expected Register Address
                    Text_Status.Text = "Success: Chip ID matched BMP180.";
                    BMPInitVals.chip_id = ReadByte[0];
                }
                else
                {
                    //Text_Status.Text = "Failed: Chip ID dosn't match BMP180.";
                    return;
                }

                tResult = I2CBmp.WriteReadPartial(CmdBuf_PromData, ReadEEPromBuff);
                if (tResult.Status == I2cTransferStatus.FullTransfer)
                {
                    // The I2C transfer succeded and we recieved the correct Chip ID from the expected Register Address
                    Text_Status.Text = "Success: Read Prom Data";
                }
                else
                {
                    Text_Status.Text = "Failed: Didnt read Prom Data";
                    return;
                }

                //
                // Fill in the Param Structure from the Prom Data.
                //
                BMPInitVals.calibParam.ac1 = (short)((ReadEEPromBuff[0] << 8) + ReadEEPromBuff[1]);
                BMPInitVals.calibParam.ac2 = (short)((ReadEEPromBuff[2] << 8) + ReadEEPromBuff[3]);
                BMPInitVals.calibParam.ac3 = (short)((ReadEEPromBuff[4] << 8) + ReadEEPromBuff[5]);
                BMPInitVals.calibParam.ac4 = (ushort)((ReadEEPromBuff[6] << 8) + ReadEEPromBuff[7]);
                BMPInitVals.calibParam.ac5 = (ushort)((ReadEEPromBuff[8] << 8) + ReadEEPromBuff[9]);
                BMPInitVals.calibParam.ac6 = (ushort)((ReadEEPromBuff[10] << 8) + ReadEEPromBuff[11]);
                BMPInitVals.calibParam.b1 =  (short)((ReadEEPromBuff[12] << 8) + ReadEEPromBuff[13]);
                BMPInitVals.calibParam.b2 =  (short)((ReadEEPromBuff[14] << 8) + ReadEEPromBuff[15]);
                BMPInitVals.calibParam.mb =  (short)((ReadEEPromBuff[16] << 8) + ReadEEPromBuff[17]);
                BMPInitVals.calibParam.mc =  (short)((ReadEEPromBuff[18] << 8) + ReadEEPromBuff[19]);
                BMPInitVals.calibParam.md =  (short)((ReadEEPromBuff[20] << 8) + ReadEEPromBuff[21]);


                periodicTimer = new Timer(this.TimerCallback, null, 1000, 2000);
            }
            /* If the write fails display the error and stop running */
            catch (Exception ex)
            {
                Text_Status.Text = "Failed to communicate with device: " + ex.Message;
                return;
            }                 
        }

        private async Task<ushort> ReadAdc(byte mux, byte gain)
        {
            Debug.WriteLine("+ReadAdc");
            //Perform an ADC read with the provided mux, gain, data_rate, and mode 
            //values.Returns the signed integer result of the read. 
            byte[] rawRes;
            try
            {
                ushort config = 0x00; // ADS1x15_CONFIG_OS_SINGLE;  // Go out of power-down mode for conversion. 
                                                           // Specify mux value. 
                config |= (ushort)((mux & 0x07) << ADS1x15_CONFIG_MUX_OFFSET);
                // Validate the passed in gain and then set it in the config. 
                //if gain not in ADS1x15_CONFIG_GAIN:
                //    raise ValueError('Gain must be one of: 2/3, 1, 2, 4, 8, 16')
                //config |= ADS1x15_CONFIG_GAIN[gain]
                config |= (ushort) ADS1115Gain.g1;

                // Set the mode (continuous or single shot). 
                //config |= mode
                // Get the default data rate if none is specified (default differs between 
                // ADS1015 and ADS1115). 
                //if data_rate is None: 
                //    data_rate = self._data_rate_default()
                //# Set the data rate (this is controlled by the subclass as it differs 
                //# between ADS1015 and ADS1115). 
                //config |= self._data_rate_config(data_rate)
                //config |= ADS1x15_CONFIG_COMP_QUE_DISABLE;  // Disble comparator mode. 
                // Send the config value to start the ADC conversion. 
                // Explicitly break the 16-bit value down to a big endian pair of bytes. 
                I2CAdc.Write(new byte[] { ADS1x15_POINTER_CONFIG, (byte)((config >> 8) & 0xFF),(byte)(config & 0xFF) });
                //.writeList(ADS1x15_POINTER_CONFIG, [(config >> 8) & 0xFF, config & 0xFF])
                //# Wait for the ADC sample to finish based on the sample rate plus a 
                //# small offset to be sure (0.1 millisecond). 
                //time.sleep(1.0 / data_rate + 0.0001)
                await Task.Delay(100);
                //# Retrieve the result. 
                //result = self._device.readList(ADS1x15_POINTER_CONVERSION, 2)
                rawRes = new byte[2];
                I2CAdc.Write(new byte[] { ADS1x15_POINTER_CONVERSION });
                I2CAdc.Read(rawRes);
                int result = (int)((short)(0xFF00 &(rawRes[1] << 8)) | 0x00ff & rawRes[0] );
                Debug.WriteLine("Adc: 0x{0:x}, {0}", result);
            }
            catch
            {
                Debug.WriteLine(" ADC I2C exception....?");
            }
            
            Debug.WriteLine("-ReadAdc");
            return 0;
        }

        private async Task<TPData> ReadI2cBarometer ()
        {
            TPData tp = new TPData();
            I2cTransferResult tResult;
            short UT = -1;

            try
            { 
                I2CBmp.Write(CmdBuf_ReadTmp); // Issue command to perform temp conversion
                await Task.Delay(BMP_TEMP_CONVERSION_TIME);
                
                tResult = I2CBmp.WriteReadPartial(CmdBuf_ReadData, ReadBuf); // read back the temp data/
               
                if (tResult.Status == I2cTransferStatus.FullTransfer)
                {
                    UT = (short)((ReadBuf[0] << 8) + ReadBuf[1]); // Got a temperatur measurement.                    
                    double TempC = CalculateCalibratedTemperature(UT, BMPInitVals.calibParam); // convet to Deg C.                     
                    tp.temperature = TempC;
                }
               
                I2CBmp.Write(CmdBuf_ReadPreas); // Issue command to perform Preasure conversion
                await Task.Delay(BMP_TEMP_CONVERSION_TIME);

                tResult = I2CBmp.WriteReadPartial(CmdBuf_ReadPreasData, ReadPresBuf); // read back the preasure data
                if (tResult.Status == I2cTransferStatus.FullTransfer)
                {
                    short oss = BMPInitVals.oversamp_setting;
                    int UP = ((ReadPresBuf[0] << 16) + (ReadPresBuf[1] << 8) + ReadPresBuf[2]) >> (8 - oss); // Got a Preasure measurement.                    
                    tp.pressure = CalculateCalibratedPreasure(UP, UT, oss, BMPInitVals.calibParam) / 100.0; // convert from hPa to kPa.                    
                }
            }
            /* If the write fails display the error and stop running */
            catch (Exception ex)
            {
                //Text_Status.Text = "Failed to communicate with device: " + ex.Message;
                throw (ex);
            }

            return tp;
        }
        private void MainPage_Unloaded(object sender, object args)
        {
            /* Cleanup */
            //I2CAccel.Dispose();
            I2CBmp.Dispose();

        }
        
        private void  TimerCallback(object state)
        {
            string xText, yText;
            string statusText;
            Task<TPData> t;
            TPData  tp;

            /* Read and format accelerometer data */
            try
            {
                t =  ReadI2cBarometer();
                tp = t.Result;
                xText = String.Format("{0:F2}", tp.temperature);
                yText = String.Format("{0:F2}", tp.pressure);
                mTemperature = xText;
                mPresure = yText;
                statusText = "Status: Running";
            }
            catch (Exception ex)
            {
                xText = "Temperature: Error";
                yText = "Pressure: Error";               
                statusText = "Failed to read from Barometer: " + ex.Message;
            }
            
            /* UI updates must be invoked on the UI thread */
            var task = this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                Text_X_Axis.Text = xText;
                Text_Y_Axis.Text = yText;
                Text_Status.Text = statusText;
            });
        }

        private void button_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e)
        {
            SendDeviceDataToCloudMessagesAsync();
        }
    }
}
