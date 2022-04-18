#include "mazeblaze.h"
#include "tuning_http_server.h"
//#include "line_following.h"
//#include "node_detection.h"
#include "turn.h"
#include "esp_err.h"
#include "wifi_logger.h"
#include <stdio.h>
#include <string.h>
// #include <sys/unistd.h>
// #include "esp_spiffs.h"
#include "esp_log.h"
//#define STRAIGHT 0

#define ST_L 1
#define PL 2
#define T 3
#define U 4
#define L 5
#define R 6
#define ST_R 7
#define E 8

bool endl = true;
int palat;
int prev = 0;
int i = 0;
char path[200] = "";
unsigned char path_length = 0;
char actual_path[] = "RRLRLLRLRRSRRLSLRLLSLRSSS";

// RRLRLLRLRRSRLRRLLRLLSLRSS
// RRLRLLRLRRSRLRRLLRLLSLRSS
// RRLRLLRLRRSRLRRLLRLLSBL

int _turn;
bool l = false, r = false, pt = false, ot = false, flag = false;
float Kp = 5;
float Ki = 0.5;
float Kd = 15 + 5;
int speed_A_0, speed_A_1;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
bool found_left = false;
bool found_straight = false;
bool found_right = false;
FILE *f;
void selection()
{
    if (_turn == ST_R || _turn == PL || _turn == T || _turn == R)
    {
        found_right = true;
        ESP_LOGI("debug", "L: %s", path);
    }
    else if (_turn == ST_L)
    {
        found_straight = true;
    }
    else if (_turn == L)
    {
        found_left = true;
    }
}
char select_turn()
{
    // selection();
    // Make a decision about how to turn.  The following code
    // implements a left-hand-on-the-wall strategy, where we always
    // turn as far to the left as possible
    if (found_left == true)
    {
        found_left = false;
        return 'L';
    }
    else if (found_straight == true)
    {
        found_straight = false;
        return 'S';
    }

    else if (found_right == true)
    {
        found_right = false;
        return 'R';
    }
    else
        return 'B';
}

void simplify_path()
{
    // only simplify the path if the second-to-last turn was a 'B'
    if (path_length < 3 || path[path_length - 2] != 'B')
        return;

    int total_angle = 0;
    int i;
    for (i = 1; i <= 3; i++)
    {
        switch (path[path_length - i])
        {
        case 'R':
            total_angle += 90;
            break;
        case 'L':
            total_angle += 270;
            break;
        case 'B':
            total_angle += 180;
            break;
        }
    }

    // Get the angle as a number between 0 and 360 degrees.
    total_angle = total_angle % 360;

    // Replace all of those turns with a single one.
    switch (total_angle)
    {
    case 0:
        path[path_length - 3] = 'S';
        break;
    case 90:
        path[path_length - 3] = 'R';
        break;
    case 180:
        path[path_length - 3] = 'B';
        break;
    case 270:
        path[path_length - 3] = 'L';
        break;
    }

    // The path is now two steps shorter.
    path_length -= 2;
    ESP_LOGI("debug", "Path: %s", path);
}

void maze_explore(void *arg)
{
    // ESP_LOGI("debug", "Initializing SPIFFS");
    // esp_vfs_spiffs_conf_t conf = {
    //     .base_path = "/spiffs",
    //     .partition_label = NULL,
    //     .max_files = 5,
    //     .format_if_mount_failed = true};

    // // Use settings defined above to initialize and mount SPIFFS filesystem.
    // // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    // esp_err_t ret = esp_vfs_spiffs_register(&conf);

    // if (ret != ESP_OK)
    // {
    //     if (ret == ESP_FAIL)
    //     {
    //         ESP_LOGE("debug", "Failed to mount or format filesystem");
    //     }
    //     else if (ret == ESP_ERR_NOT_FOUND)
    //     {
    //         ESP_LOGE("debug", "Failed to find SPIFFS partition");
    //     }
    //     else
    //     {
    //         ESP_LOGE("debug", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
    //     }
    //     return;
    // }
    // size_t total = 0, used = 0;
    // ret = esp_spiffs_info(conf.partition_label, &total, &used);
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE("debug", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    // }
    // else
    // {
    //     ESP_LOGI("debug", "Partition size: total: %d, used: %d", total, used);
    // }
    while (1)
    {
        if (gpio_get_level(DEBUG_SWITCH) == 0)
        {
            int duty_cycle = 75;

            //        ESP_LOGI("debug", "ACTUAL: %d", _turn);
            // ESP_LOGI("debug", "Reading file");
            // f = fopen("/spiffs/hello.txt", "r");
            // if (f == NULL)
            // {
            //     ESP_LOGE("debug", "Failed to open file for reading");
            //     return;
            // }
            // char line[64];
            // fgets(line, sizeof(line), f);
            // fclose(f);
            // // strip newline
            // char *pos = strchr(line, '\n');
            // if (pos)
            // {
            //     *pos = '\0';
            // }
            // ESP_LOGI("debug", "Read from file: '%s'", line);
            // // esp_vfs_spiffs_unregister(conf.partition_label);
            // ESP_LOGI("debug", "SPIFFS unmounted");

            set_led_off();
            while (1)
            {
                if ((read_lsa().data[1] == 1) && (read_lsa().data[2] == 0) && (read_lsa().data[3] == 0))
                    error = -2;
                else if ((read_lsa().data[1] == 1) && (read_lsa().data[2] == 1) && (read_lsa().data[3] == 0))
                    error = -1;
                else if ((read_lsa().data[1] == 0) && (read_lsa().data[2] == 1) && (read_lsa().data[3] == 0))
                    error = 0;
                else if ((read_lsa().data[1] == 0) && (read_lsa().data[2] == 1) && (read_lsa().data[3] == 1))
                    error = 1;
                else if ((read_lsa().data[1] == 0) && (read_lsa().data[2] == 0) && (read_lsa().data[3] == 1))
                    error = 2;
                P = error;
                I = I + previous_I;
                D = error - previous_error;
                PID_value = (Kp * P) + (Ki * I) + (Kd * D);
                previous_I = I;
                previous_error = error;
                speed_A_0 = duty_cycle - PID_value;
                speed_A_1 = duty_cycle + PID_value;
                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, speed_A_0);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, speed_A_1);
                prev++;
                if (((read_lsa().data[4] == 1) || (read_lsa().data[0] == 1)) && (prev >= 50))
                {
                    prev = 0;
                    break;
                }
                // if ((read_lsa().data[1] == 1) && (read_lsa().data[2] == 0) && (read_lsa().data[3] == 1))
                // {
                //     _turn = U;
                //     palat = UTURN;
                //     break;
                // }
            }
            if (read_lsa().data[4] == 1 || read_lsa().data[0] == 1)
            {
                vTaskDelay(50 / portTICK_PERIOD_MS);

                while (i <= 100)
                {
                    ESP_LOGI("debug", "PATH: %d", i);

                    if (actual_path[i] == 'R')
                    {
                        prev = 0;
                        take_turn(RIGHT);
                        i++;
                        break;
                    }
                    else if (actual_path[i] == 'L')
                    {
                        take_turn(LEFT);
                        prev = 0;
                        i++;

                        break;
                    }
                    else if (actual_path[i] == 'S')
                    {
                        take_turn(STRAIGHT);
                        prev = 0;
                        i++;

                        break;
                    }
                }
            }
            if (i > 24)
            {
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 75);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 75);
                set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
                set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 75);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 75);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
                set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
                complete = true;
                set_led_on();
                vTaskDelay(20000 / portTICK_PERIOD_MS);

                break;

                // set_led_off();
                // vTaskDelay(100 / portTICK_RATE_MS);
            }
            // ESP_LOGI("debug", "Turn: %d", _turn);
            // ESP_LOGI("debug", "Palat: %d", palat);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else
        {
            int duty_cycle = 73;

            while (1)
            {
                if ((read_lsa().data[1] == 1) && (read_lsa().data[2] == 0) && (read_lsa().data[3] == 0))
                    error = -2;
                else if ((read_lsa().data[1] == 1) && (read_lsa().data[2] == 1) && (read_lsa().data[3] == 0))
                    error = -1;
                else if ((read_lsa().data[1] == 0) && (read_lsa().data[2] == 1) && (read_lsa().data[3] == 0))
                    error = 0;
                else if ((read_lsa().data[1] == 0) && (read_lsa().data[2] == 1) && (read_lsa().data[3] == 1))
                    error = 1;
                else if ((read_lsa().data[1] == 0) && (read_lsa().data[2] == 0) && (read_lsa().data[3] == 1))
                    error = 2;
                P = error;
                I = I + previous_I;
                D = error - previous_error;
                PID_value = (Kp * P) + (Ki * I) + (Kd * D);
                previous_I = I;
                previous_error = error;
                speed_A_0 = duty_cycle - PID_value;
                speed_A_1 = duty_cycle + PID_value;
                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, speed_A_0);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, speed_A_1);
                prev++;
                if (((read_lsa().data[4] == 1) || (read_lsa().data[0] == 1)) && (prev >= 50))
                {
                    prev = 0;
                    break;
                }
                if ((read_lsa().data[1] == 1) && (read_lsa().data[2] == 0) && (read_lsa().data[3] == 1))
                {
                    _turn = U;
                    palat = UTURN;
                    break;
                }
            }
            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 62);
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 62);
            vTaskDelay(30 / portTICK_PERIOD_MS);
            set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
            set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
            if ((read_lsa().data[4] == 1) && (read_lsa().data[0] == 1))
            {
                pt = true;
            }
            else if ((read_lsa().data[4] == 1) && (read_lsa().data[0] == 0))
            {
                l = true;
            }
            else if ((read_lsa().data[4] == 0) && (read_lsa().data[0] == 1))
            {
                r = true;
            }
            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 62);
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 62);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
            set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
            if (pt)
            {
                _turn = T;
                palat = RIGHT;
            }
            if (r)
            {
                _turn = R;
                palat = RIGHT;
                // if (read_lsa().data[2] == 1 && read_lsa().data[0] == 1)
                // {
                //     _turn = ST_R;
                //     set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 75);
                //     set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 75);
                //     vTaskDelay(350 / portTICK_PERIOD_MS);
                //     set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 85);
                //     set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 85);
                //     vTaskDelay(150 / portTICK_PERIOD_MS);
                //     set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 75);
                //     set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 75);
                //     vTaskDelay(300 / portTICK_PERIOD_MS);
                //     set_led_on();
                //     vTaskDelay(100 / portTICK_RATE_MS);
                //     set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
                //     set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
                //     vTaskDelay(2000 / portTICK_PERIOD_MS);
                //     // First create a file.
                //     ESP_LOGI("debug", "Opening file");
                //     f = fopen("/spiffs/hello.txt", "w");
                //     if (f == NULL)
                //     {
                //         ESP_LOGE("debug", "Failed to open file for writing");
                //         // return;
                //     }
                //     fputs(path, f);
                //     fclose(f);
                //     ESP_LOGI("debug", "File written");
            }
            if (l)
            {
                if (read_lsa().data[2] == 0)
                {
                    _turn = L;
                    palat = LEFT;
                }
                else
                {
                    _turn = ST_L;
                    palat = STRAIGHT;

                    // set_led_off();
                    // vTaskDelay(100 / portTICK_RATE_MS);
                }
            }
        }
        l = false;
        r = false;
        pt = false;
        // if (complete)
        // {
        //     // First create a file.
        //     ESP_LOGI("debug", "Opening file");
        //     f = fopen("/spiffs/hello.txt", "w");
        //     if (f == NULL)
        //     {
        //         ESP_LOGE("debug", "Failed to open file for writing");
        //         // return;
        //     }
        //     fputs(path, f);
        //     fclose(f);
        //     ESP_LOGI("debug", "File written");
        //     complete = false;
        // }
        // ESP_LOGI("debug", "Turn: %d", _turn);
        // ESP_LOGI("debug", "Palat: %d", palat);

        if (_turn != E)
        {
            ESP_LOGI("debug", "Itterations: %d", prev);
            prev = 0;
            // set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
            // set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
            take_turn(palat);
            ESP_LOGI("debug", "Turn: %d", _turn);

            selection();
            unsigned char dir = select_turn();
            path[path_length] = dir;
            path_length++;

            // You should check to make sure that the path_length does not
            // exceed the bounds of the array.  We'll ignore that in this
            // example.

            // Simplify the learned path.
            simplify_path();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(enable_lsa());
    ESP_ERROR_CHECK(enable_led());
    ESP_ERROR_CHECK(enable_motor_driver());

    // Use POSIX and C standard library functions to work with files.

    start_wifi_logger(); // Start wifi logger

    if (endl)
    {
        xTaskCreate(&start_tuning_http_server, "start server", 4096, NULL, 1, NULL);
        xTaskCreate(&maze_explore, "maze explore", 8192, NULL, 1, &Maze_explore);
    }
}