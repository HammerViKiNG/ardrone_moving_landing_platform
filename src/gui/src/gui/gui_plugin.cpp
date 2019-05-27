#include "gui/gui_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QDebug>

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
    std::string topic_image = "/ardrone/image_raw",
                topic_navdata = "/ardrone/navdata";

    controller = nullptr;

    ongoing = false;

    ros::NodeHandle nh_ = getNodeHandle();
    image_transport::ImageTransport it_(nh_);

    ui_.image_frame->setOuterLayout(ui_.image_layout);
    
    image_sub_ = it_.subscribe(topic_image, 1, std::bind(&GUIPlugin::callbackImage, this, std::placeholders::_1, ui_.image_frame));
    sub_navdata_ = nh_.subscribe(topic_navdata, 10, &GUIPlugin::callbackNavdata, this);

    connect(ui_.s_high, SIGNAL(valueChanged(int)), this, SLOT(setNecessaryHeight(int)));
    connect(ui_.start_button, SIGNAL(clicked(void)), this, SLOT(startSequence(void)));
    connect(ui_.stop_button, SIGNAL(clicked(void)), this, SLOT(stopSequence(void)));
}

  
void GUIPlugin::shutdownPlugin()
{
    image_sub_.shutdown();
    sub_navdata_.shutdown();
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
    std::lock_guard<std::mutex> lock(mutex_ardrone);
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
            }
            else if (msg->encoding == "8UC1")
            {
            // convert gray to rgb
                cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
            }
        }
        catch (cv_bridge::Exception& e)
        {
            image_frame->setImage(QImage());
            return;
        }
    }
    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
    image_frame->setImage(image);
}


void GUIPlugin::setNecessaryHeight(int value)
{
    ui_.curr_s_high->setText(QString::number(value / sliderCoeff));
    if (controller != nullptr)
    {
        std::lock_guard<std::mutex> lock(mutex_ardrone);
        this->controller->set_necessary_height(value / sliderCoeff);
    }
}


void GUIPlugin::setFlightMode()
{
    if (controller != nullptr)
    {
        std::lock_guard<std::mutex> lock(mutex_ardrone);
        this->controller->set_is_hovering(ui_.rb_hovering->isChecked());
    }
}


void GUIPlugin::startSequence()
{
    if (this->controller == nullptr)
    {
        this->ongoing = true;
        ardrone_thread = new std::thread([this]() {
            this->controller = new ArdroneARTag("/ardrone/navdata", "/cmd_vel", "/ardrone/ar_tag_front", "/ardrone/ar_tag_bottom", "/gui_control", 200);
            ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
            ros::Rate rate(200);
            while (this->ongoing)
            {
                mutex_ardrone.lock();
                this->controller->control();
                mutex_ardrone.unlock();
                ros::spinOnce();
                rate.sleep();
            }
        } );
    }
}


void GUIPlugin::stopSequence()
{
    if (ardrone_thread != nullptr)
    {
        std::lock_guard<std::mutex> lock(mutex_ardrone);
        this->ongoing = false;
        this->ardrone_thread = nullptr;
        this->controller = nullptr;
    }
}


void GUIPlugin::callbackNavdata(const ardrone_autonomy::Navdata& msg)
{
    std::lock_guard<std::mutex> lock(mutex_ardrone);
    uint8_t rounding_digit = 1;
    ui_.v_label->setText(QString( "vx: ") + QString::number( round_digit(msg.vx, rounding_digit) ) 
                       + QString( " vy: ") + QString::number( round_digit(msg.vy, rounding_digit) ) 
                       + QString( " vz: ") + QString::number( round_digit(msg.vz, rounding_digit) ));
    ui_.a_label->setText(QString( "ax: ") + QString::number( round_digit(msg.ax, rounding_digit) ) 
                       + QString( " ay: ") + QString::number( round_digit(msg.ay, rounding_digit) ) 
                       + QString( " az: ") + QString::number( round_digit(msg.az, rounding_digit) ));
    ui_.rot_label->setText(QString( " rot_x: ") + QString::number( round_digit(msg.rotX, rounding_digit) ) 
                         + QString( " rot_y: ") + QString::number( round_digit(msg.rotY, rounding_digit) ) 
                         + QString( " rot_z: ") + QString::number( round_digit(msg.rotZ, rounding_digit) ));
}



float GUIPlugin::round_digit(float number, uint8_t digit)
{
    return round(number * pow(10, digit)) / pow(10.0f, digit);
}

} // namespace

PLUGINLIB_DECLARE_CLASS(gui, GUIPlugin, gui::GUIPlugin, rqt_gui_cpp::Plugin);
