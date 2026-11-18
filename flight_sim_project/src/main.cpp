#include "chu_init.h"
#include "gpio_cores.h"
#include "spi_core.h"
#include "vga_core.h"
#include <cmath>
#include <cstdint>

// Instantiate SPI, switches, and VGA cores
SpiCore spi(get_slot_addr(BRIDGE_BASE, S9_SPI));
GpiCore sw(get_slot_addr(BRIDGE_BASE, S3_SW));
GpvCore horizon(get_sprite_addr(BRIDGE_BASE, V5_USER5));
GpvCore mountain(get_sprite_addr(BRIDGE_BASE, V4_USER4));

// ADXL362 registers
const uint8_t RD_CMD = 0x0B;
const uint8_t WR_CMD = 0x0A;
const uint8_t PART_ID_REG = 0x02;
const uint8_t XDATA_L_REG = 0x0E;
const uint8_t POWER_CTL_REG = 0x2D;

// VGA horizon registers
const int PITCH_REG = 0x2001;
const int ROLL_REG = 0x2002;

// VGA mountain registers
const int MOUNTAIN_FACE_BASE = 0x2001;
const int MOUNTAIN_FACE_STRIDE = 11;
const int MOUNTAIN_COMMIT_REG = 0x2059;

// Mountain geometry
const int MOUNTAIN_COUNT = 2;
const int VERTICES_PER_MOUNTAIN = 5;
const int FACES_PER_MOUNTAIN = 4;
const int TOTAL_VERTICES = 10;
const int TOTAL_FACES = 8;

// Flight control constants
const float PI = 3.14159265f;
const float RAD_TO_DEG = 180.0f / PI;
const float DEG_TO_RAD = PI / 180.0f;

const float FILTER_ALPHA = 0.20f;
const float MAX_PITCH = 20.0f;
const float MAX_ROLL = 35.0f;
const float DEADZONE = 1.0f;

// VGA scaling
const float PITCH_SCALE = 5.0f;
const float ROLL_SCALE = 256.0f;

// Flight simulation constants
const float DT = 0.02f;
const float GRAVITY = 9.81f;
const float KNOT_TO_MPS = 0.514444f;
const float METER_TO_FEET = 3.28084f;

const float MIN_AIRSPEED = 60.0f;
const float MAX_AIRSPEED = 180.0f;

// Initial aircraft conditions
const float START_X = -600.0f;
const float START_Z = 0.0f;
const float START_ALTITUDE = 4000.0f;
const float START_HEADING = 0.0f;

// Mountain projection constants
const float FOCAL_LENGTH = 360.0f;
const float MIN_DEPTH = 60.0f;

// Mountain 1 world coordinates
const float M1_X = 0.0f;
const float M1_Z = 2000.0f;
const float M1_HEIGHT = 1900.0f;
const float M1_RADIUS = 500.0f;

// Mountain 2 world coordinates
const float M2_X = -1350.0f;
const float M2_Z = 3200.0f;
const float M2_HEIGHT = 1450.0f;
const float M2_RADIUS = 450.0f;

// Aircraft state
struct AircraftState
{
    float x;
    float z;

    float pitch;
    float roll;
    float heading;

    float altitude;
    float airspeed;
    float vertical_speed;
};

// 3D world vertex
struct Vertex3D
{
    float x;
    float y;
    float z;
};

// Projected VGA vertex
struct Vertex2D
{
    int32_t x;
    int32_t y;
};

// Ten vertices for two 3D mountains
const Vertex3D MOUNTAIN_VERTICES[TOTAL_VERTICES] =
{
    // Mountain 1
    {M1_X - M1_RADIUS, 0.0f, M1_Z - M1_RADIUS},
    {M1_X + M1_RADIUS, 0.0f, M1_Z - M1_RADIUS},
    {M1_X + M1_RADIUS, 0.0f, M1_Z + M1_RADIUS},
    {M1_X - M1_RADIUS, 0.0f, M1_Z + M1_RADIUS},
    {M1_X, M1_HEIGHT, M1_Z},

    // Mountain 2
    {M2_X - M2_RADIUS, 0.0f, M2_Z - M2_RADIUS},
    {M2_X + M2_RADIUS, 0.0f, M2_Z - M2_RADIUS},
    {M2_X + M2_RADIUS, 0.0f, M2_Z + M2_RADIUS},
    {M2_X - M2_RADIUS, 0.0f, M2_Z + M2_RADIUS},
    {M2_X, M2_HEIGHT, M2_Z}
};

// Eight triangular faces
const int FACE_INDICES[TOTAL_FACES][3] =
{
    // Mountain 1
    {0, 1, 4},
    {1, 2, 4},
    {2, 3, 4},
    {3, 0, 4},

    // Mountain 2
    {5, 6, 9},
    {6, 7, 9},
    {7, 8, 9},
    {8, 5, 9}
};

// Mountain face colors (12-bit RGB)
const uint16_t FACE_COLORS[TOTAL_FACES] =
{
    // Mountain 1 - green
    0x797,
    0x465,
    0x576,
    0x8A8,

    // Mountain 2 - brown-gray
    0xA87,
    0x654,
    0x765,
    0xB98
};

// Initialize ADXL362
bool gsensor_init(SpiCore *spi_p)
{
    int id;

    spi_p->set_freq(400000);
    spi_p->set_mode(0, 0);

    // Read part ID
    spi_p->assert_ss(0);
    spi_p->transfer(RD_CMD);
    spi_p->transfer(PART_ID_REG);
    id = spi_p->transfer(0x00);
    spi_p->deassert_ss(0);

    uart.disp("ADXL362 ID: ");
    uart.disp(id, 16);
    uart.disp("\n\r");

    if (id != 0xF2)
    {
        uart.disp("Accelerometer not detected!\n\r");
        return false;
    }

    // Enable measurement mode
    spi_p->assert_ss(0);
    spi_p->transfer(WR_CMD);
    spi_p->transfer(POWER_CTL_REG);
    spi_p->transfer(0x02);
    spi_p->deassert_ss(0);

    sleep_ms(20);

    return true;
}

// Read acceleration data
void read_acceleration(SpiCore *spi_p,
                       float *ax, float *ay, float *az)
{
    uint8_t data[6];

    spi_p->assert_ss(0);
    spi_p->transfer(RD_CMD);
    spi_p->transfer(XDATA_L_REG);

    for (int i = 0; i < 6; i++)
    {
        data[i] = spi_p->transfer(0x00);
    }

    spi_p->deassert_ss(0);

    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);

    // Convert raw acceleration to g
    *ax = x / 1000.0f;
    *ay = y / 1000.0f;
    *az = z / 1000.0f;
}

// Calculate pitch and roll
void calculate_orientation(float ax, float ay, float az,
                           float *pitch, float *roll)
{
    *roll = atan2(ay, -az) * RAD_TO_DEG;

    *pitch = atan2(-ax, sqrt(ay * ay + az * az))
             * RAD_TO_DEG;
}

// Limit a value
float clamp_value(float value, float min_value, float max_value)
{
    if (value < min_value) { return min_value; }
    if (value > max_value) { return max_value; }

    return value;
}

// Apply control deadzone
float apply_deadzone(float value)
{
    if (fabs(value) < DEADZONE)
    {
        return 0.0f;
    }

    return value;
}

// Calibrate accelerometer
void calibrate_controls(SpiCore *spi_p,
                        float *pitch_offset, float *roll_offset)
{
    float ax, ay, az;
    float pitch, roll;
    float pitch_sum = 0.0f;
    float roll_sum = 0.0f;

    uart.disp("Keep the FPGA board level...\n\r");
    sleep_ms(1000);

    for (int i = 0; i < 50; i++)
    {
        read_acceleration(spi_p, &ax, &ay, &az);
        calculate_orientation(ax, ay, az, &pitch, &roll);

        pitch_sum += pitch;
        roll_sum += roll;

        sleep_ms(20);
    }

    *pitch_offset = pitch_sum / 50.0f;
    *roll_offset = roll_sum / 50.0f;

    uart.disp("Calibration complete!\n\r");
}

// Read SW0-SW3 for airspeed
float read_airspeed(GpiCore *sw_p)
{
    int switch_value = 0;

    for (int i = 0; i < 4; i++)
    {
        if (sw_p->read(i))
        {
            switch_value += (1 << i);
        }
    }

    // Map 0-15 to 60-180 knots
    float airspeed = MIN_AIRSPEED +
                     switch_value * (MAX_AIRSPEED -
                                     MIN_AIRSPEED) / 15.0f;

    return airspeed;
}

// Update aircraft position and heading
void update_flight_dynamics(AircraftState *aircraft)
{
    float pitch_rad = aircraft->pitch * DEG_TO_RAD;
    float roll_rad = aircraft->roll * DEG_TO_RAD;
    float heading_rad = aircraft->heading * DEG_TO_RAD;

    // Convert airspeed to meters per second
    float velocity = aircraft->airspeed * KNOT_TO_MPS;

    // Simplified coordinated turn rate
    float turn_rate = GRAVITY * tan(roll_rad) / velocity;

    // Update heading
    heading_rad += turn_rate * DT;

    // Normalize heading
    aircraft->heading = heading_rad * RAD_TO_DEG;

    while (aircraft->heading >= 360.0f)
    {
        aircraft->heading -= 360.0f;
    }

    while (aircraft->heading < 0.0f)
    {
        aircraft->heading += 360.0f;
    }

    // Vertical velocity
    float climb_rate = velocity * sin(pitch_rad);

    // Update altitude in feet
    aircraft->altitude += climb_rate * DT * METER_TO_FEET;

    // Vertical speed in feet per minute
    aircraft->vertical_speed = climb_rate *
                               METER_TO_FEET * 60.0f;

    // Prevent negative altitude
    if (aircraft->altitude < 0.0f)
    {
        aircraft->altitude = 0.0f;
        aircraft->vertical_speed = 0.0f;
    }

    // Horizontal movement
    float horizontal_velocity = velocity * cos(pitch_rad);

    aircraft->x += horizontal_velocity *
                   sin(heading_rad) * DT;

    aircraft->z += horizontal_velocity *
                   cos(heading_rad) * DT;
}

// Update VGA artificial horizon
void update_horizon(float pitch, float roll)
{
    int32_t pitch_pixels;
    int32_t roll_slope;

    // Pitch direction matches physical board movement
    pitch_pixels = (int32_t)(-pitch * PITCH_SCALE);

    // Q8 fixed-point roll slope
    roll_slope = (int32_t)(-tan(roll * DEG_TO_RAD)
                          * ROLL_SCALE);

    horizon.wr_mem(PITCH_REG, (uint32_t)pitch_pixels);
    horizon.wr_mem(ROLL_REG, (uint32_t)roll_slope);
}

// Project 3D world vertex onto VGA
bool project_vertex(Vertex3D vertex,
                    AircraftState *aircraft,
                    Vertex2D *screen)
{
    float heading_rad = aircraft->heading * DEG_TO_RAD;
    float roll_rad = aircraft->roll * DEG_TO_RAD;

    float cos_heading = cos(heading_rad);
    float sin_heading = sin(heading_rad);

    float cos_roll = cos(roll_rad);
    float sin_roll = sin(roll_rad);

    // Aircraft altitude in meters
    float altitude_m = aircraft->altitude / METER_TO_FEET;

    // Vertex relative to aircraft
    float dx = vertex.x - aircraft->x;
    float dy = vertex.y - altitude_m;
    float dz = vertex.z - aircraft->z;

    // Rotate into camera coordinates
    float camera_x = cos_heading * dx
                   - sin_heading * dz;

    float camera_z = sin_heading * dx
                   + cos_heading * dz;

    // Reject vertices behind near plane
    if (camera_z <= MIN_DEPTH)
    {
        return false;
    }

    // Perspective projection
    float projected_x = FOCAL_LENGTH * camera_x / camera_z;
    float projected_y = -FOCAL_LENGTH * dy / camera_z;

    // Rotate projected coordinates with aircraft roll
    float rotated_x = cos_roll * projected_x
                    + sin_roll * projected_y;

    float rotated_y = -sin_roll * projected_x
                    + cos_roll * projected_y;

    // Match artificial horizon pitch direction
    float pitch_offset = -aircraft->pitch * PITCH_SCALE;

    float screen_x = 320.0f + rotated_x;
    float screen_y = 240.0f + pitch_offset + rotated_y;

    // Limit projected coordinates
    screen_x = clamp_value(screen_x, -2048.0f, 2048.0f);
    screen_y = clamp_value(screen_y, -2048.0f, 2048.0f);

    screen->x = (int32_t)screen_x;
    screen->y = (int32_t)screen_y;

    return true;
}

// Write one triangle's edge coefficients to FPGA
void write_mountain_face(int face,
                         Vertex2D *v,
                         uint16_t color)
{
    int addr = MOUNTAIN_FACE_BASE +
               face * MOUNTAIN_FACE_STRIDE;

    // Calculate three edge equations
    for (int e = 0; e < 3; e++)
    {
        int next = (e + 1) % 3;

        int32_t A = v[e].y - v[next].y;
        int32_t B = v[next].x - v[e].x;

        int64_t C =
            (int64_t)v[e].x * v[next].y
          - (int64_t)v[next].x * v[e].y;

        // Write edge coefficients
        mountain.wr_mem(addr + e * 3 + 0,
                        (uint32_t)A);

        mountain.wr_mem(addr + e * 3 + 1,
                        (uint32_t)B);

        mountain.wr_mem(addr + e * 3 + 2,
                        (uint32_t)(int32_t)C);
    }

    // Set face color
    mountain.wr_mem(addr + 9, color);

    // Enable face
    mountain.wr_mem(addr + 10, 1);
}

// Disable a mountain face
void disable_mountain_face(int face)
{
    int addr = MOUNTAIN_FACE_BASE +
               face * MOUNTAIN_FACE_STRIDE;

    mountain.wr_mem(addr + 10, 0);
}

// Update two 3D mountains
void update_mountain(AircraftState *aircraft)
{
    Vertex2D projected[TOTAL_VERTICES];
    bool visible[TOTAL_VERTICES];

    // Project all ten world vertices
    for (int i = 0; i < TOTAL_VERTICES; i++)
    {
        visible[i] = project_vertex(
            MOUNTAIN_VERTICES[i], aircraft, &projected[i]);
    }

    // Update all eight triangular faces
    for (int f = 0; f < TOTAL_FACES; f++)
    {
        int i0 = FACE_INDICES[f][0];
        int i1 = FACE_INDICES[f][1];
        int i2 = FACE_INDICES[f][2];

        // Disable faces crossing near plane
        if (!visible[i0] || !visible[i1] || !visible[i2])
        {
            disable_mountain_face(f);
            continue;
        }

        Vertex2D v[3] =
        {
            projected[i0],
            projected[i1],
            projected[i2]
        };

        // Calculate signed triangle area
        int64_t area =
            (int64_t)(v[1].x - v[0].x) *
                      (v[2].y - v[0].y)
          - (int64_t)(v[1].y - v[0].y) *
                      (v[2].x - v[0].x);

        // Back-face culling
        if (area >= 0)
        {
            disable_mountain_face(f);
            continue;
        }

        // Write visible mountain face
        write_mountain_face(f, v, FACE_COLORS[f]);
    }

    // Commit all eight faces at VGA frame origin
    mountain.wr_mem(MOUNTAIN_COMMIT_REG, 1);

    // Enable mountain renderer
    mountain.bypass(0);
}

// Reset aircraft to starting flight conditions
void reset_flight(AircraftState *aircraft)
{
    // Starting position
    aircraft->x = START_X;
    aircraft->z = START_Z;

    aircraft->pitch = 0.0f;
    aircraft->roll = 0.0f;
    aircraft->heading = START_HEADING;

    aircraft->altitude = START_ALTITUDE;
    aircraft->airspeed = MIN_AIRSPEED;
    aircraft->vertical_speed = 0.0f;

    // Reset VGA graphics
    update_horizon(0.0f, 0.0f);
    update_mountain(aircraft);

    uart.disp("\n\rFlight simulation reset!\n\r");
}

// Main flight simulation
void flight_simulator(SpiCore *spi_p, GpiCore *sw_p)
{
    AircraftState aircraft = {};

    // Initial aircraft conditions
    aircraft.x = START_X;
    aircraft.z = START_Z;

    aircraft.pitch = 0.0f;
    aircraft.roll = 0.0f;
    aircraft.heading = START_HEADING;

    aircraft.altitude = START_ALTITUDE;
    aircraft.airspeed = MIN_AIRSPEED;
    aircraft.vertical_speed = 0.0f;

    float ax, ay, az;
    float pitch, roll;
    float pitch_offset, roll_offset;
    float target_pitch, target_roll;

    int print_counter = 0;

    // SW15 - leftmost switch
    int reset_switch;
    int previous_reset = 0;

    // Calibrate accelerometer
    calibrate_controls(spi_p, &pitch_offset, &roll_offset);

    // Initialize artificial horizon
    update_horizon(0.0f, 0.0f);
    horizon.bypass(0);

    // Initialize both mountains
    update_mountain(&aircraft);

    uart.disp("Flight simulator active!\n\r");
    uart.disp("Two 3D mountains enabled!\n\r");
    uart.disp("SW0-SW3 = Airspeed (60-180 knots)\n\r");
    uart.disp("SW15 ON = Reset and Hold\n\r");
    uart.disp("SW15 OFF = Resume Flight\n\r");

    while (1)
    {
        // Check SW15 reset switch
        reset_switch = sw_p->read(15);

        // Reset and hold while SW15 is ON
        if (reset_switch == 1)
        {
            // Reset only once when SW15 is activated
            if (previous_reset == 0)
            {
                reset_flight(&aircraft);
                print_counter = 0;
            }

            previous_reset = 1;

            // Freeze aircraft position and orientation
            sleep_ms(20);
            continue;
        }

        // Resume flight when SW15 is OFF
        previous_reset = 0;

        // Read accelerometer
        read_acceleration(spi_p, &ax, &ay, &az);

        // Calculate board orientation
        calculate_orientation(ax, ay, az, &pitch, &roll);

        // Remove initial board orientation
        target_pitch = pitch - pitch_offset;

        // Correct roll direction
        target_roll = -(roll - roll_offset);

        // Limit aircraft orientation
        target_pitch = clamp_value(target_pitch,
                                   -MAX_PITCH, MAX_PITCH);

        target_roll = clamp_value(target_roll,
                                  -MAX_ROLL, MAX_ROLL);

        // Smooth aircraft movement
        aircraft.pitch += FILTER_ALPHA *
                          (target_pitch - aircraft.pitch);

        aircraft.roll += FILTER_ALPHA *
                         (target_roll - aircraft.roll);

        // Apply deadzone
        aircraft.pitch = apply_deadzone(aircraft.pitch);
        aircraft.roll = apply_deadzone(aircraft.roll);

        // Read airspeed from SW0-SW3
        aircraft.airspeed = read_airspeed(sw_p);

        // Update flight dynamics
        update_flight_dynamics(&aircraft);

        // Update VGA display
        update_horizon(aircraft.pitch, aircraft.roll);
        update_mountain(&aircraft);

        // Print UART telemetry
        if (print_counter >= 5)
        {
            uart.disp("Pitch: ");
            uart.disp((double)aircraft.pitch, 2);

            uart.disp(" | Roll: ");
            uart.disp((double)aircraft.roll, 2);

            uart.disp(" | Heading: ");
            uart.disp((double)aircraft.heading, 1);

            uart.disp(" | Alt: ");
            uart.disp((double)aircraft.altitude, 1);

            uart.disp(" | Speed: ");
            uart.disp((double)aircraft.airspeed, 0);

            uart.disp(" | VS: ");
            uart.disp((double)aircraft.vertical_speed, 0);

            uart.disp("\n\r");

            print_counter = 0;
        }

        print_counter++;

        sleep_ms(20);
    }
}

int main()
{
    uart.disp("\n\rFPGA Flight Simulator\n\r");
    uart.disp("Initializing flight simulation...\n\r");

    if (!gsensor_init(&spi))
    {
        while (1) {}
    }

    flight_simulator(&spi, &sw);

    return 0;
}
