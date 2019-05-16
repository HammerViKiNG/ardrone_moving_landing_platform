#include "gui/gui_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace gui {

GUIPlugin::GUIPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
    setObjectName("GUIPlugin");
}

void GUIPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
    QStringList argv = context.argv();
  // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);
    std::string topic_front = "/ardrone/front/image_raw",
                topic_bottom = "/ardrone/bottom/image_raw";
    image_transport::ImageTransport it_(getNodeHandle());

    ui_.front_image_frame->setOuterLayout(ui_.front_layout);
    ui_.bottom_image_frame->setOuterLayout(ui_.bottom_layout);
    
    front_sub_ = it_.subscribe(topic_front, 1, boost::bind(&GUIPlugin::callbackImage, this, _1, ui_.front_image_frame));
    bottom_sub_ = it_.subscribe(topic_bottom, 1, boost::bind(&GUIPlugin::callbackImage, this, _1, ui_.bottom_image_frame));
}
  
void GUIPlugin::shutdownPlugin()
{
    front_sub_.shutdown();
    bottom_sub_.shutdown();
}
  
void GUIPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    // TODO save intrinsic configuration, usually using:
    // instance_settings.setValue(k, v)
}

void GUIPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    // TODO restore intrinsic configuration, usually using:
    // v = instance_settings.value(k)
}
 
/*bool hasConfiguration() const
{
    return true;
}

void triggerConfiguration()
{
    // Usually used to open a dialog to offer the user a set of configuration
}*/

void GUIPlugin::callbackImage(const sensor_msgs::Image::ConstPtr& msg, rqt_image_view::RatioLayoutedFrame* image_frame)
{
  try
  {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
        // scale / quantify
        image_frame->setImage(QImage());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      image_frame->setImage(QImage());
      return;
    }
  }
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
  image_frame->setImage(image);
}

} // namespace

PLUGINLIB_DECLARE_CLASS(gui, GUIPlugin, gui::GUIPlugin, rqt_gui_cpp::Plugin);
