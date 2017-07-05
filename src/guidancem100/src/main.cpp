#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"  
#include <fstream>
#include <iostream>
#include "DJI_guidance.h"
#include "DJI_utility.h"
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace cv;
using namespace std;

DJI_event g_event;
DJI_lock g_lock;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)
#define RETURN_IF_ERR(err_code) { if( err_code ){ printf( "USB error code:%d in file %s %d\n", err_code, __FILE__, __LINE__ );}}
#define RELEASE_IF_ERR(err_code) { if( err_code ){ release_transfer(); printf( "USB error code:%d in file %s %d\n", err_code, __FILE__, __LINE__ );}}

int on_data(int data_type, int data_len, char *content);

class Camera {
    public:
        string name;
        e_vbus_index bus;
        bool is_grayscale_subscribed;
        bool is_depth_subscribed;
        Mat left;
        Mat right;
        Mat depth;
        Mat displayDepth;
        VideoWriter imageWriter;
        VideoWriter depthWriter;
        Camera();
        Camera(string name);
        Camera(string name, e_vbus_index evbus);

        void subscribe_grayscale();
        void subscribe_depth();
        void convert_depth();
        void record();
};

Camera::Camera(){
}

Camera::Camera(string a, e_vbus_index evbus){
    name = a;
    bus = evbus;
    left = Mat(HEIGHT, WIDTH, CV_8U);
    right = Mat(HEIGHT, WIDTH, CV_8U);
    depth = Mat(HEIGHT, WIDTH, CV_16SC1);
    displayDepth = Mat(HEIGHT, WIDTH, CV_8UC1);
    imageWriter = VideoWriter("videos/" + name + "-image.avi", CV_FOURCC('M','J','P','G'), 25, Size(WIDTH, HEIGHT), false);
    depthWriter = VideoWriter("videos/" + name + "-depth.avi", CV_FOURCC('M','J','P','G'), 25, Size(WIDTH, HEIGHT), false);

    is_grayscale_subscribed = false;
    is_depth_subscribed = false;
}

void Camera::subscribe_grayscale() {
    int err = select_greyscale_image(bus, true);
    RELEASE_IF_ERR(err);

    err = select_greyscale_image(bus, false);
    RELEASE_IF_ERR(err);

    is_grayscale_subscribed = true;
}

void Camera::subscribe_depth() {
    int err = select_depth_image(bus);
    RELEASE_IF_ERR(err);
    is_depth_subscribed = true;
}

void Camera::convert_depth() {
    depth.convertTo(displayDepth, CV_8UC1);
}

void Camera::record() {
    if(is_grayscale_subscribed) {
        imageWriter << left;
    }

    if(is_depth_subscribed) {
        depthWriter << displayDepth;
    }
}

class M100 {
    public:
        Camera one;
        Camera two;
        Camera three;
        Camera four;
        Camera five;

        M100();
        void create_windows();
        void reset();
        void init();
        void subscribe_grayscale(int cam);
        void subscribe_depth(int cam);
        void set_rate(e_image_data_frequecy frequency);
        void start();
        void stop();
        void release();
};

M100::M100(){
    one = Camera("one", e_vbus1);
    two = Camera("two", e_vbus2);
    three = Camera("three", e_vbus3);
    four = Camera("four", e_vbus4);
}

void M100::create_windows() {
    namedWindow("w1");
    namedWindow("w2");
    // namedWindow("w3");
    // namedWindow("w4");
    //
    // namedWindow("d1");
    // namedWindow("d2");
    // namedWindow("d3");
    // namedWindow("d4");
}

void M100::reset() {
    reset_config();
}

void M100::init() {
    int err = init_transfer();
    RETURN_IF_ERR(err);
}

void M100::subscribe_grayscale(int cam) {
    switch(cam) {
        case 1:
            one.subscribe_grayscale();
            break;
        case 2:
            two.subscribe_grayscale();
            break;
        case 3:
            three.subscribe_grayscale();
            break;
        case 4:
            four.subscribe_grayscale();
            break;
    }
}

void M100::subscribe_depth(int cam) {
    switch(cam) {
        case 1:
            one.subscribe_depth();
            break;
        case 2:
            two.subscribe_depth();
            break;
        case 3:
            three.subscribe_depth();
            break;
        case 4:
            four.subscribe_depth();
            break;
    }
}

void M100::set_rate(e_image_data_frequecy frequency) {
    int err = set_image_frequecy(frequency);
    RELEASE_IF_ERR(err);
}

void M100::start() {
    int err = start_transfer();
    RELEASE_IF_ERR(err);
}

void M100::stop() {
    int err = stop_transfer();
    RELEASE_IF_ERR(err);

    /*Make sure that the data transfer has stopped*/
    sleep(100000);
}

void M100::release() {
    int err = release_transfer();
    RELEASE_IF_ERR(err);
}

M100 bot;

int on_data(int data_type, int data_len, char *content) {
    g_lock.enter();
    if ( e_image == data_type && NULL != content )
    {
        image_data data;
        memcpy((char*)&data, content, sizeof(data));

        if(bot.one.is_grayscale_subscribed) {
            memcpy(bot.one.left.data, data.m_greyscale_image_left[e_vbus1], IMAGE_SIZE);
            memcpy(bot.one.right.data, data.m_greyscale_image_right[e_vbus1], IMAGE_SIZE);
        }

        if(bot.one.is_depth_subscribed) {
            memcpy(bot.one.depth.data, data.m_depth_image[e_vbus1], IMAGE_SIZE * 2);
        }

        if(bot.two.is_grayscale_subscribed) {
            memcpy(bot.two.left.data, data.m_greyscale_image_left[e_vbus1], IMAGE_SIZE);
            memcpy(bot.two.right.data, data.m_greyscale_image_right[e_vbus1], IMAGE_SIZE);
        }

        if(bot.two.is_depth_subscribed) {
            memcpy(bot.two.depth.data, data.m_depth_image[e_vbus1], IMAGE_SIZE * 2);
        }

        if(bot.three.is_grayscale_subscribed) {
            memcpy(bot.three.left.data, data.m_greyscale_image_left[e_vbus1], IMAGE_SIZE);
            memcpy(bot.three.right.data, data.m_greyscale_image_right[e_vbus1], IMAGE_SIZE);
        }

        if(bot.three.is_depth_subscribed) {
            memcpy(bot.three.depth.data, data.m_depth_image[e_vbus1], IMAGE_SIZE * 2);
        }

        if(bot.four.is_grayscale_subscribed) {
            memcpy(bot.four.left.data, data.m_greyscale_image_left[e_vbus1], IMAGE_SIZE);
            memcpy(bot.four.right.data, data.m_greyscale_image_right[e_vbus1], IMAGE_SIZE);
        }

        if(bot.four.is_depth_subscribed) {
            memcpy(bot.four.depth.data, data.m_depth_image[e_vbus1], IMAGE_SIZE * 2);
        }
    }
    g_lock.leave();
    g_event.set_event();
    return 0;
}

void run() {
    printf("Create Windows...");
    bot.create_windows();

    printf("Reset and initialize...");
    bot.reset();
    bot.init();

    printf("Subscribe to depth data...");
    bot.one.subscribe_depth();
    bot.two.subscribe_depth();
    // bot.three.subscribe_grayscale();
    // bot.four.subscribe_grayscale();

    printf("set callback function...");
    int err = set_sdk_event_handler(on_data);
    RELEASE_IF_ERR(err);

    printf("Start transfer...");
    bot.start();

    for(int times = 0; times < 200; ++times) {
        printf("For %d...", times);
        g_event.wait_event();

        bot.one.convert_depth();
        bot.two.convert_depth();

        imshow("w1", bot.one.displayDepth);
        imshow("w2", bot.two.displayDepth);

        bot.one.record();
        bot.two.record();
    }

    bot.stop();
    bot.release();
}

int main( int argc, const char** argv )
{
    run();
    return 0;
}
