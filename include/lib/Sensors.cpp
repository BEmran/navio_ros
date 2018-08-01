#include "Sensors.h"

Sensors::Sensors (){}


Sensors::Sensors (std::string sensor_name, bool debug = true){
    is_debug = debug;
    bias.gx = 0.0;
    bias.gy = 0.0;
    bias.gz = 0.0;

    // get current time
    getTime();

    if (sensor_name == "mpu") {
        printf("Selected: MPU9250\n");
        is = new MPU9250();
    }
    else if (sensor_name == "lsm") {
        printf("Selected: LSM9DS1\n");
        is = new LSM9DS1();
    }
    else {
        is = NULL;
    }

    // Create a file to store the row data
    if (is_debug){
        char file_name[128];
        sprintf(file_name,"row_data_%s.txt", sensor_name.c_str());
        row_data_file = fopen(file_name, "w");
        printf("Start storing the data in the file \"%s\"\n",file_name);
        fprintf(row_data_file, "time, ax, ay, az, gx, gy, gz, mx, my, mz\n");
    }

    // Initilaize imu sensor and calibrate gyro
    is->initialize();
    isISEnabled = is->probe();
    if (isISEnabled){
        calibrateGyro();
    }
}

void Sensors::update(){
    is->update();
    is->read_accelerometer(&imu.ax, &imu.ay, &imu.az);
    is->read_gyroscope(&imu.gx, &imu.gy, &imu.gz);
    is->read_magnetometer(&imu.mx, &imu.my, &imu.mz);
    // rotate axis
    float tmpax = imu.ax;
    float tmpgx = imu.gx;
    float tmpmx = imu.mx;
    imu.ax = -imu.ay;
    imu.gx = -imu.gy;
    imu.mx = -imu.my;
    imu.ay = -tmpax;
    imu.gy = -tmpgx;
    imu.my = -tmpmx;

    // apply calibration
    imu.ax /= G_SI;
    imu.ay /= G_SI;
    imu.az /= G_SI;

    imu.gx -= bias.gx;
    imu.gy -= bias.gy;
    imu.gz -= bias.gz;

    // store data
    if (is_debug){
        storeData();
    }

}

//**************************************************************************
// Calibrate Gyroscope sensor: find bias values
//**************************************************************************
void Sensors::calibrateGyro()
{
    //---------------------- Calculate the offset -----------------------------
    float offset[3] = {0.0, 0.0, 0.0};

    //-------------------------------------------------------------------------
    printf("Beginning Gyro calibration...\n");
    for(int i = 0; i<500; i++)
    {
        update();

        imu.gx *= 180 / PI;
        imu.gy *= 180 / PI;
        imu.gz *= 180 / PI;

        offset[0] += imu.gx * 0.0175;
        offset[1] += imu.gy * 0.0175;
        offset[2] += imu.gz * 0.0175;

        usleep(5000);
    }
    offset[0]/=500.0;
    offset[1]/=500.0;
    offset[2]/=500.0;

    printf("Offsets are: %+10.5f %+10.5f %+10.5f\n", offset[0], offset[1], offset[2]);

    bias.gx = offset[0];
    bias.gx = offset[1];
    bias.gx = offset[2];
}
//**************************************************************************
// Get Initial Orientation: find the initial orientation
//**************************************************************************
void Sensors::getInitialOrientation()
{
    //---------------------- Calculate the offset -----------------------------
    float avg[3] = {0.0, 0.0, 0.0};

    //-------------------------------------------------------------------------
    printf("Beginning Orientation calibration...\n");
    for(int i = 0; i<500; i++)
    {
        update();

        avg[0] += imu.ax;
        avg[1] += imu.ay;
        avg[2] += imu.az;

        usleep(5000);
    }
    avg[0]/=500.0;
    avg[1]/=500.0;
    avg[2]/=500.0;

    printf("Orientation Offsets are: %+10.5f %+10.5f %+10.5f\n", avg[0], avg[1], avg[2]);

    init_Orient[0] = avg[0];
    init_Orient[1] = avg[1];
    init_Orient[2] = avg[2];
}
//**************************************************************************
// Store row measurements
//**************************************************************************

void Sensors::storeData() {
    // get current time
    getTime();
    // Write data
    fprintf(row_data_file, "%12lu,",time_now);
    fprintf(row_data_file, " %+10.5f,  %+10.5f,  %+10.5f,",imu.ax, imu.ay, imu.az);
    fprintf(row_data_file, " %+10.5f,  %+10.5f,  %+10.5f,",imu.gx, imu.gy, imu.gz);
    fprintf(row_data_file, " %+10.5f,  %+10.5f,  %+10.5f\n",imu.mx, imu.my, imu.mz);
}
//**************************************************************************
// Get the current time
//**************************************************************************

void Sensors::getTime() {
    // Timing data
    struct timeval tv;

    // Calculate delta time
    gettimeofday(&tv, NULL);
    time_now = 1000000 * tv.tv_sec + tv.tv_usec;
}
