#ifndef gui_plugin_H
#define gui_plugin_H

#include "ros/ros.h"
#include <thread>
#include <mutex>

#include <rqt_gui_cpp/plugin.h>
#include <gui/ui_gui_plugin.h>

#include <sensor_msgs/Image.h>
#include "ardrone_autonomy/Navdata.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QString>
#include <QWidget>

#include "ardrone_ar_tag/ardrone_ar_tag_control.h"

namespace gui {
 
    class GUIPlugin
     : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
    public:
        GUIPlugin();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    protected:
        virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg, rqt_image_view::RatioLayoutedFrame* image_frame);
        virtual void callbackNavdata(const ardrone_autonomy::Navdata& msg);

    protected slots:
        virtual void setNecessaryHeight(int value);
        virtual void setFlightMode();
        virtual void startSequence();
        virtual void stopSequence();
  
        // Comment in to signal that the plugin has a way to configure it
        //bool hasConfiguration() const;
        //void triggerConfiguration();
    private:
        static float round_digit(float number, uint8_t digit);

        const float sliderCoeff = 10.0f;

        Ui::GUI ui_;
        QWidget* widget_;

        image_transport::Subscriber front_sub_, bottom_sub_;
        cv::Mat conversion_mat_;

        ros::Subscriber sub_navdata_;

        std::thread *ardrone_thread;
        std::mutex mutex_ardrone;

        bool ongoing;

        ArdroneARTag *controller;
  };
} // namespace
#endif // gui_plugin_H
   
