#ifndef gui_plugin_H
#define gui_plugin_H

#include "ros/ros.h"

#include <rqt_gui_cpp/plugin.h>
#include <gui/ui_gui_plugin.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>
#include <QString>
#include <QWidget>

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
  
        // Comment in to signal that the plugin has a way to configure it
        //bool hasConfiguration() const;
        //void triggerConfiguration();
    private:
        Ui::GUI ui_;
        QWidget* widget_;

        image_transport::Subscriber front_sub_, bottom_sub_;
        cv::Mat conversion_mat_;
  };
} // namespace
#endif // gui_plugin_H
   
