#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "image_editor/ParameterConfig.h"
#include "image_editor/image_editor.h"

namespace image_editor {
class ImageEditorNodelet : public nodelet::Nodelet {
 public:
    virtual void onInit() {
        image_editor_nodelet_ptr_ =
            std::make_unique<ImageEditor>(getNodeHandle(), getPrivateNodeHandle());
        dr_server_ptr_ =
            std::make_unique<dynamic_reconfigure::Server<image_editor::ParameterConfig>>(
                getPrivateNodeHandle());
        dynamic_reconfigure::Server<image_editor::ParameterConfig>::CallbackType dr_callback;
        dr_callback = boost::bind(&ImageEditorNodelet::update_dynamic_reconfigure, this, _1, _2);
        dr_server_ptr_->setCallback(dr_callback);
        roomba = image_editor_nodelet_ptr_->get_roomba();
    }

    void update_dynamic_reconfigure(image_editor::ParameterConfig &config, uint32_t level) {
        if (roomba == "roomba1") {
            image_editor_nodelet_ptr_->set_threshold(config.roomba1_whitebalance_threshold);
        } else if (roomba == "roomba2") {
            image_editor_nodelet_ptr_->set_threshold(config.roomba2_whitebalance_threshold);
        } else if (roomba == "roomba3") {
            image_editor_nodelet_ptr_->set_threshold(config.roomba3_whitebalance_threshold);
        } else if (roomba == "roomba4") {
            image_editor_nodelet_ptr_->set_threshold(config.roomba4_whitebalance_threshold);
        } else if (roomba == "roomba5") {
            image_editor_nodelet_ptr_->set_threshold(config.roomba5_whitebalance_threshold);
        } else if (roomba == "roomba6") {
            image_editor_nodelet_ptr_->set_threshold(config.roomba6_whitebalance_threshold);
        }
    }

 private:
    std::unique_ptr<ImageEditor> image_editor_nodelet_ptr_;
    std::unique_ptr<dynamic_reconfigure::Server<image_editor::ParameterConfig>> dr_server_ptr_;

    std::string roomba;
    std::array<double, 6> whitebalance_thresholds;
};
}  // namespace image_editor
PLUGINLIB_EXPORT_CLASS(image_editor::ImageEditorNodelet, nodelet::Nodelet);
