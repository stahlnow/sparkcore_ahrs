#include "AHRS.h"

#include "application.h"
#include <cmath>

float AHRS::accel[3] = {0, 0, 0};  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float AHRS::accel_min[3] = {0, 0, 0};
float AHRS::accel_max[3] = {0, 0, 0};

float AHRS::magnetom[3] = {0, 0, 0};
float AHRS::magnetom_min[3] = {0, 0, 0};
float AHRS::magnetom_max[3] = {0, 0, 0};
float AHRS::magnetom_tmp[3] = {0, 0, 0};

float AHRS::gyro[3] = {0, 0, 0};
float AHRS::gyro_average[3] = {0, 0, 0};

float AHRS::Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float AHRS::Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float AHRS::Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float AHRS::Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float AHRS::Omega_I[3]= {0, 0, 0}; // Omega Integrator
float AHRS::Omega[3]= {0, 0, 0};
float AHRS::errorRollPitch[3] = {0, 0, 0};
float AHRS::errorYaw[3] = {0, 0, 0};

float AHRS::DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float AHRS::Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float AHRS::Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

AHRS::AHRS(int16_t output_mode, int16_t output_format, uint16_t status_led_pin, bool output_errors)
: output_mode(output_mode), output_format(output_format), status_led_pin(status_led_pin), output_errors(output_errors)
{
    // Init status LED
    pinMode (status_led_pin, OUTPUT);
    digitalWrite(status_led_pin, LOW);

    I2C_Init();
    Accel_Init();
    Magn_Init();
    Gyro_Init();

    // Read sensors, init DCM algorithm
    delay(20); // Give sensors enough time to collect data
    reset_sensor_fusion();


    // Init output
    #if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
    turn_output_stream_off();
    #else
    turn_output_stream_on();
    #endif

}

AHRS::~AHRS()
{

}


// Read incoming control messages
void AHRS::check_commands()
{
    if (Serial.available() >= 2)
    {
        if (Serial.read() == '#') // Start of new control message
        {
            int16_t command = Serial.read(); // Commands
            if (command == 'f') // request one output _f_rame
                setOutputSingleOn(true);
            else if (command == 's') // _s_ynch request
            {
                // Read ID
                byte id[2];
                id[0] = readChar();
                id[1] = readChar();

                // Reply with synch message
                Serial.print("#SYNCH");
                Serial.write(id, 2);
                Serial.println();
            }
            else if (command == 'o') // Set _o_utput mode
            {
                char output_param = readChar();
                if (output_param == 'n')  // Calibrate _n_ext sensor
                {
                    curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
                    reset_calibration_session_flag = true;
                }
                else if (output_param == 't') // Output angles as _t_ext
                {
                    output_mode = OUTPUT__MODE_ANGLES;
                    output_format = OUTPUT__FORMAT_TEXT;
                }
                else if (output_param == 'b') // Output angles in _b_inary format
                {
                    output_mode = OUTPUT__MODE_ANGLES;
                    output_format = OUTPUT__FORMAT_BINARY;
                }
                else if (output_param == 'c') // Go to _c_alibration mode
                {
                    output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
                    reset_calibration_session_flag = true;
                }
                else if (output_param == 's') // Output _s_ensor values
                {
                    char values_param = readChar();
                    char format_param = readChar();
                    if (values_param == 'r')  // Output _r_aw sensor values
                        output_mode = OUTPUT__MODE_SENSORS_RAW;
                    else if (values_param == 'c')  // Output _c_alibrated sensor values
                        output_mode = OUTPUT__MODE_SENSORS_CALIB;
                    else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
                        output_mode = OUTPUT__MODE_SENSORS_BOTH;

                    if (format_param == 't') // Output values as _t_text
                        output_format = OUTPUT__FORMAT_TEXT;
                    else if (format_param == 'b') // Output values in _b_inary format
                        output_format = OUTPUT__FORMAT_BINARY;
                }
                else if (output_param == '0') // Disable continuous streaming output
                {
                    turn_output_stream_off();
                    reset_calibration_session_flag = true;
                }
                else if (output_param == '1') // Enable continuous streaming output
                {
                    reset_calibration_session_flag = true;
                    turn_output_stream_on();
                }
                else if (output_param == 'e') // _e_rror output settings
                {
                    char error_param = readChar();
                    if (error_param == '0') output_errors = false;
                    else if (error_param == '1') output_errors = true;
                    else if (error_param == 'c') // get error count
                    {
                        Serial.print("#AMG-ERR:");
                        Serial.print(num_accel_errors); Serial.print(",");
                        Serial.print(num_magn_errors); Serial.print(",");
                        Serial.println(num_gyro_errors);
                    }
                }
            }

        }
        else
        {
        } // Skip character
    }
}

void AHRS::run()
{
    // Time to read the sensors again?
    if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
    {
        timestamp_old = timestamp;
        timestamp = millis();
        if (timestamp > timestamp_old)
            G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
        else G_Dt = 0;

        // Update sensor readings
        read_sensors();

        if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
        {
            check_reset_calibration_session();  // Check if this session needs a reset
            if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
        }
        else if (output_mode == OUTPUT__MODE_ANGLES)  // Output angles
        {
            // Apply sensor calibration
            compensate_sensor_errors();

            // Run DCM algorithm
            Compass_Heading(); // Calculate magnetic heading
            Matrix_update();
            Normalize();
            Drift_correction();
            Euler_angles();

            if (output_stream_on || output_single_on) output_angles();
        }
        else  // Output sensor values
        {
            if (output_stream_on || output_single_on) output_sensors();
        }

        output_single_on = false;

        #if DEBUG__PRINT_LOOP_TIME == true
        Serial.print("loop time (ms) = ");
        Serial.println(millis() - timestamp);
        #endif
    }

    #if DEBUG__PRINT_LOOP_TIME == true
    else
    {
        Serial.println("waiting...");
    }
    #endif
}

void AHRS::read_sensors()
{
    Read_Gyro(); // Read gyroscope
    Read_Accel(); // Read accelerometer
    Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void AHRS::reset_sensor_fusion()
{
    float temp1[3];
    float temp2[3];
    float xAxis[] = {1.0f, 0.0f, 0.0f};

    read_sensors();
    timestamp = millis();

    // GET PITCH
    // Using y-z-plane-component/x-component of gravity vector
    pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

    // GET ROLL
    // Compensate pitch of gravity vector
    Vector_Cross_Product(temp1, accel, xAxis);
    Vector_Cross_Product(temp2, xAxis, temp1);
     // Normally using x-z-plane-component/y-component of compensated gravity vector
    // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
    // Since we compensated for pitch, x-z-plane-component equals z-component:
    roll = atan2(temp2[1], temp2[2]);

    // GET YAW
    Compass_Heading();
    yaw = MAG_Heading;

    // Init rotation matrix
    init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void AHRS::compensate_sensor_errors()
{
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
    for (int16_t i = 0; i < 3; i++)
        magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void AHRS::check_reset_calibration_session()
{
    // Raw sensor values have to be read already, but no error compensation applied

    // Reset this calibration session?
    if (!reset_calibration_session_flag) return;

    // Reset acc and mag calibration variables
    for (int16_t i = 0; i < 3; i++) {
        accel_min[i] = accel_max[i] = accel[i];
        magnetom_min[i] = magnetom_max[i] = magnetom[i];
    }

    // Reset gyro calibration variables
    gyro_num_samples = 0;  // Reset gyro calibration averaging
    gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;

    reset_calibration_session_flag = false;
}

void AHRS::turn_output_stream_on()
{
    output_stream_on = true;
    digitalWrite(status_led_pin, HIGH);
}

void AHRS::turn_output_stream_off()
{
    output_stream_on = false;
    digitalWrite(status_led_pin, LOW);
}

// Blocks until another byte is available on serial port
char AHRS::readChar()
{
    while (Serial.available() < 1) { } // Block
        return Serial.read();
}


void AHRS::Compass_Heading()
{
    float mag_x;
    float mag_y;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;

    cos_roll = cos(roll);
    sin_roll = sin(roll);
    cos_pitch = cos(pitch);
    sin_pitch = sin(pitch);

    // Tilt compensated magnetic field X
    mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
    // Tilt compensated magnetic field Y
    mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
    // Magnetic Heading
    MAG_Heading = atan2(-mag_y, mag_x);
}


/********************************************************************************
 DCM algorithm
********************************************************************************/

 void AHRS::Normalize(void)
 {
    float error=0;
    float temporary[3][3];
    float renorm=0;

    error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

    Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
    Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

    Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
    Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

    Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20

    renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
    Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

    renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
    Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

    renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
    Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}


void AHRS::Drift_correction(void)
{
    float mag_heading_x;
    float mag_heading_y;
    float errorCourse;
    //Compensation the Roll, Pitch and Yaw drift.
    static float Scaled_Omega_P[3];
    static float Scaled_Omega_I[3];
    float Accel_magnitude;
    float Accel_weight;


    //*****Roll and Pitch***************

    // Calculate the magnitude of the accelerometer vector
    Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
    Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
    // Dynamic weighting of accelerometer info (reliability filter)
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //

    Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
    Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);

    Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);

    //*****YAW***************
    // We make the gyro YAW drift correction based on compass magnetic heading

    mag_heading_x = cos(MAG_Heading);
    mag_heading_y = sin(MAG_Heading);
    errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}

void AHRS::Matrix_update(void)
{
    Gyro_Vector[0]=GYRO_SCALED_RAD(gyro[0]); //gyro x roll
    Gyro_Vector[1]=GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
    Gyro_Vector[2]=GYRO_SCALED_RAD(gyro[2]); //gyro z yaw

    Accel_Vector[0]=accel[0];
    Accel_Vector[1]=accel[1];
    Accel_Vector[2]=accel[2];

    Vector_Add(Omega, Gyro_Vector, Omega_I);  //adding proportional term
    Vector_Add(Omega_Vector, Omega, Omega_P); //adding Integrator term

#if DEBUG__NO_DRIFT_CORRECTION == true // Do not use drift correction
    Update_Matrix[0][0]=0;
    Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
    Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
    Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
    Update_Matrix[1][1]=0;
    Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
    Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
    Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
    Update_Matrix[2][2]=0;
#else // Use drift correction
    Update_Matrix[0][0]=0;
    Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
    Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
    Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
    Update_Matrix[1][1]=0;
    Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
    Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
    Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
    Update_Matrix[2][2]=0;
#endif

    Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c

    for(int16_t x=0; x<3; x++) //Matrix Addition (update)
    {
        for(int16_t y=0; y<3; y++)
        {
            DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
        }
    }
}

void AHRS::Euler_angles(void)
{
    pitch = -asin(DCM_Matrix[2][0]);
    roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}


/********************************************************************************
 Math stuff
********************************************************************************/

// Computes the dot product of two vectors
 float AHRS::Vector_Dot_Product(const float v1[3], const float v2[3])
 {
    float result = 0;

    for(int16_t c = 0; c < 3; c++)
    {
        result += v1[c] * v2[c];
    }

    return result;
}

// Computes the cross product of two vectors
// out has to different from v1 and v2 (no in-place)!
void AHRS::Vector_Cross_Product(float out[3], const float v1[3], const float v2[3])
{
    out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
    out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
    out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

// Multiply the vector by a scalar
void AHRS::Vector_Scale(float out[3], const float v[3], float scale)
{
    for(int16_t c = 0; c < 3; c++)
    {
        out[c] = v[c] * scale;
    }
}

// Adds two vectors
void AHRS::Vector_Add(float * out, const float * v1, const float * v2)
{
    for(int16_t c = 0; c < 3; c++)
    {
        out[c] = v1[c] + v2[c];
    }
}

// Multiply two 3x3 matrices: out = a * b
// out has to different from a and b (no in-place)!
void AHRS::Matrix_Multiply(const float a[][3], const float b[][3], float out[][3])
{

    for(int16_t x = 0; x < 3; x++)  // rows
    {
        for(int16_t y = 0; y < 3; y++)  // columns
        {
            out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
        }
    }
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void AHRS::Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
    for(int16_t x = 0; x < 3; x++)
    {
        out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
    }
}

// Init rotation matrix using euler angles
void AHRS::init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll)
{
    float c1 = cos(roll);
    float s1 = sin(roll);
    float c2 = cos(pitch);
    float s2 = sin(pitch);
    float c3 = cos(yaw);
    float s3 = sin(yaw);

    // Euler angles, right-handed, intrinsic, XYZ convention
    // (which means: rotate around body axes Z, Y', X'')
    m[0][0] = c2 * c3;
    m[0][1] = c3 * s1 * s2 - c1 * s3;
    m[0][2] = s1 * s3 + c1 * c3 * s2;

    m[1][0] = c2 * s3;
    m[1][1] = c1 * c3 + s1 * s2 * s3;
    m[1][2] = c1 * s2 * s3 - c3 * s1;

    m[2][0] = -s2;
    m[2][1] = c2 * s1;
    m[2][2] = c1 * c2;
}


/********************************************************************************
 Output
********************************************************************************/

// Output angles: yaw, pitch, roll
 void AHRS::output_angles()
 {
    if (output_format == OUTPUT__FORMAT_BINARY)
    {
        float ypr[3];
        ypr[0] = TO_DEG(yaw);
        ypr[1] = TO_DEG(pitch);
        ypr[2] = TO_DEG(roll);
        Serial.write((byte*) ypr, 12);  // No new-line
    }
    else if (output_format == OUTPUT__FORMAT_TEXT)
    {
        Serial.print("#YPR=");
        Serial.print(TO_DEG(yaw)); Serial.print(",");
        Serial.print(TO_DEG(pitch)); Serial.print(",");
        Serial.print(TO_DEG(roll)); Serial.println();
    }
}

void AHRS::output_calibration(int16_t calibration_sensor)
{
    if (calibration_sensor == 0)  // Accelerometer
    {
        // Output MIN/MAX values
        Serial.print("accel x,y,z (min/max) = ");
        for (int16_t i = 0; i < 3; i++) {
            if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
            if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
            Serial.print(accel_min[i]);
            Serial.print("/");
            Serial.print(accel_max[i]);
            if (i < 2) Serial.print("  ");
            else Serial.println();
        }
    }
    else if (calibration_sensor == 1)  // Magnetometer
    {
        // Output MIN/MAX values
        Serial.print("magn x,y,z (min/max) = ");
        for (int16_t i = 0; i < 3; i++) {
            if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
            if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
            Serial.print(magnetom_min[i]);
            Serial.print("/");
            Serial.print(magnetom_max[i]);
            if (i < 2) Serial.print("  ");
            else Serial.println();
        }
    }
    else if (calibration_sensor == 2)  // Gyroscope
    {
        // Average gyro values
        for (int16_t i = 0; i < 3; i++)
            gyro_average[i] += gyro[i];
        gyro_num_samples++;

        // Output current and averaged gyroscope values
        Serial.print("gyro x,y,z (current/average) = ");
        for (int16_t i = 0; i < 3; i++) {
            Serial.print(gyro[i]);
            Serial.print("/");
            Serial.print(gyro_average[i] / (float) gyro_num_samples);
            if (i < 2) Serial.print("  ");
            else Serial.println();
        }
    }
}

void AHRS::output_sensors_text(char raw_or_calibrated)
{
    Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
    Serial.print(accel[0]); Serial.print(",");
    Serial.print(accel[1]); Serial.print(",");
    Serial.print(accel[2]); Serial.println();

    Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
    Serial.print(magnetom[0]); Serial.print(",");
    Serial.print(magnetom[1]); Serial.print(",");
    Serial.print(magnetom[2]); Serial.println();

    Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
    Serial.print(gyro[0]); Serial.print(",");
    Serial.print(gyro[1]); Serial.print(",");
    Serial.print(gyro[2]); Serial.println();
}

void AHRS::output_sensors_binary()
{
    Serial.write((byte*) accel, 12);
    Serial.write((byte*) magnetom, 12);
    Serial.write((byte*) gyro, 12);
}

void AHRS::output_sensors()
{
    if (output_mode == OUTPUT__MODE_SENSORS_RAW)
    {
        if (output_format == OUTPUT__FORMAT_BINARY)
            output_sensors_binary();
        else if (output_format == OUTPUT__FORMAT_TEXT)
            output_sensors_text('R');
    }
    else if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
    {
        // Apply sensor calibration
        compensate_sensor_errors();

        if (output_format == OUTPUT__FORMAT_BINARY)
            output_sensors_binary();
        else if (output_format == OUTPUT__FORMAT_TEXT)
            output_sensors_text('C');
    }
    else if (output_mode == OUTPUT__MODE_SENSORS_BOTH)
    {
        if (output_format == OUTPUT__FORMAT_BINARY)
        {
            output_sensors_binary();
            compensate_sensor_errors();
            output_sensors_binary();
        }
        else if (output_format == OUTPUT__FORMAT_TEXT)
        {
            output_sensors_text('R');
            compensate_sensor_errors();
            output_sensors_text('C');
        }
    }
}




/********************************************************************************
I2C code to read the sensors
********************************************************************************/

void AHRS::I2C_Init()
{
    Wire.begin();
}

void AHRS::Accel_Init()
{
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(0x2D);  // Power register
    Wire.write(0x08);  // Measurement mode
    Wire.endTransmission();
    delay(5);
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(0x31);  // Data format register
    Wire.write(0x08);  // Set to full resolution
    Wire.endTransmission();
    delay(5);
    // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(0x2C);  // Rate
    Wire.write(0x09);  // Set to 50Hz, normal operation
    Wire.endTransmission();
    delay(5);
}

// Reads x, y and z accelerometer registers
void AHRS::Read_Accel()
{
    int16_t i = 0;
    byte buff[6];

    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.write(0x32);  // Send address to read from
    Wire.endTransmission();

    Wire.beginTransmission(ACCEL_ADDRESS);
    Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
    while(Wire.available())  // ((Wire.available())&&(i<6))
    {
        buff[i] = Wire.read();  // Read one byte
        i++;
    }
    Wire.endTransmission();

    if (i == 6)  // All bytes received?
    {
        // No multiply by -1 for coordinate system transformation here, because of double negation:
        // We want the gravity vector, which is negated acceleration vector.
        int16_t a0 = (buff[3] << 8) | buff[2];  // X axis (internal sensor y axis)
        int16_t a1 = (buff[1] << 8) | buff[0];  // Y axis (internal sensor x axis)
        int16_t a2 = (buff[5] << 8) | buff[4];  // Z axis (internal sensor z axis)
        accel[0] = (float)a0;
        accel[1] = (float)a1;
        accel[2] = (float)a2;
    }
    else
    {
        num_accel_errors++;
        if (output_errors) Serial.println("!ERR: reading accelerometer");
    }
}

void AHRS::Magn_Init()
{
    Wire.beginTransmission(MAGN_ADDRESS);
    Wire.write(0x02);
    Wire.write(0x00);  // Set continuous mode (default 10Hz)
    Wire.endTransmission();
    delay(5);

    Wire.beginTransmission(MAGN_ADDRESS);
    Wire.write(0x00);
    Wire.write(0b00011000);  // Set 50Hz
    Wire.endTransmission();
    delay(5);
}

void AHRS::Read_Magn()
{
    int16_t i = 0;
    byte buff[6];

    Wire.beginTransmission(MAGN_ADDRESS);
    Wire.write(0x03);  // Send address to read from
    Wire.endTransmission();

    Wire.beginTransmission(MAGN_ADDRESS);
    Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
    while(Wire.available())  // ((Wire.available())&&(i<6))
    {
        buff[i] = Wire.read();  // Read one byte
        i++;
    }
    Wire.endTransmission();

    if (i == 6)  // All bytes received?
    {
        // 9DOF Razor IMU SEN-10125 using HMC5843 magnetometer
    #if HW__VERSION_CODE == 10125
    // MSB byte first, then LSB; X, Y, Z
        int16_t m0 = -1 * ((((int16_t) buff[2]) << 8) | buff[3]);  // X axis (internal sensor -y axis)
        int16_t m1 = -1 * ((((int16_t) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
        int16_t m2 = -1 * ((((int16_t) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
        magnetom[0] = (float)m0;
        magnetom[1] = (float)m1;
        magnetom[2] = (float)m2;
        // 9DOF Razor IMU SEN-10736 using HMC5883L magnetometer
    #elif HW__VERSION_CODE == 10736
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
        int16_t m0 = -1 * ((((int16_t) buff[4]) << 8) | buff[5]);  // X axis (internal sensor -y axis)
        int16_t m1 = -1 * ((((int16_t) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
        int16_t m2 = -1 * ((((int16_t) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
        magnetom[0] = (float)m0;
        magnetom[1] = (float)m1;
        magnetom[2] = (float)m2;
    // 9DOF Sensor Stick SEN-10183 and SEN-10321 using HMC5843 magnetometer
    #elif (HW__VERSION_CODE == 10183) || (HW__VERSION_CODE == 10321)
    // MSB byte first, then LSB; X, Y, Z
        int16_t m0 = (((int16_t) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
        int16_t m1 = -1 * ((((int16_t) buff[2]) << 8) | buff[3]);  // Y axis (internal sensor -y axis)
        int16_t m2 = -1 * ((((int16_t) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
        magnetom[0] = (float)m0;
        magnetom[1] = (float)m1;
        magnetom[2] = (float)m2;
    // 9DOF Sensor Stick SEN-10724 using HMC5883L magnetometer
    #elif HW__VERSION_CODE == 10724
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
        int16_t m0 = (((int16_t) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
        int16_t m1 = -1 * ((((int16_t) buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
        int16_t m2 = -1 * ((((int16_t) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
        magnetom[0] = (float)m0;
        magnetom[1] = (float)m1;
        magnetom[2] = (float)m2;
    #endif
    }
    else
    {
        num_magn_errors++;
        if (output_errors) Serial.println("!ERR: reading magnetometer");
    }
}

void AHRS::Gyro_Init()
{
    // Power up reset defaults
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x3E);
    Wire.write(0x80);
    Wire.endTransmission();
    delay(5);

    // Select full-scale range of the gyro sensors
    // Set LP filter bandwidth to 42Hz
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x16);
    Wire.write(0x1B);  // DLPF_CFG = 3, FS_SEL = 3
    Wire.endTransmission();
    delay(5);

    // Set sample rato to 50Hz
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x15);
    Wire.write(0x0A);  //  SMPLRT_DIV = 10 (50Hz)
    Wire.endTransmission();
    delay(5);

    // Set clock to PLL with z gyro reference
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x3E);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(5);
}

// Reads x, y and z gyroscope registers
void AHRS::Read_Gyro()
{
    int16_t i = 0;
    byte buff[6];

    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x1D);  // Sends address to read from
    Wire.endTransmission();

    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
    while(Wire.available())  // ((Wire.available())&&(i<6))
    {
        buff[i] = Wire.read();  // Read one byte
        i++;
    }
    Wire.endTransmission();

    if (i == 6)  // All bytes received?
    {
        int16_t g0 = -1 * ((((int16_t) buff[2]) << 8) | buff[3]);    // X axis (internal sensor -y axis)
        int16_t g1 = -1 * ((((int16_t) buff[0]) << 8) | buff[1]);    // Y axis (internal sensor -x axis)
        int16_t g2 = -1 * ((((int16_t) buff[4]) << 8) | buff[5]);    // Z axis (internal sensor -z axis)

        gyro[0] = (float)g0;
        gyro[1] = (float)g1;
        gyro[2] = (float)g2;

    }
    else
    {
        num_gyro_errors++;
        if (output_errors) Serial.println("!ERR: reading gyroscope");
    }
}
