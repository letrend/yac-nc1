#include <cnc_gui/cnc_gui.hpp>

CNCGUI::CNCGUI()
        : widget_(0) {
        this->setObjectName("CNCGUI");
}

void CNCGUI::initPlugin(qt_gui_cpp::PluginContext &context) {
        // access standalone command line arguments
        QStringList argv = context.argv();
        // create QWidget
        widget_ = new QWidget();
        // extend the widget with all attributes and children from UI file
        ui.setupUi(widget_);
        // add widget to the user interface
        context.addWidget(widget_);

        nh = ros::NodeHandlePtr(new ros::NodeHandle);
        if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "CNCGUI");
        }

        string configFile;
        nh->getParam("brainDiceConfigFile", configFile);
        if(!readConfig(configFile)) {
                brainDiceConfigFile = QFileDialog::getOpenFileName(widget_,
                                                                   tr("Select brain dice config file"), motorConfigFile,
                                                                   tr("brain dice config file (*.yaml)"));
                if(!readConfig(brainDiceConfigFile.toStdString())) {
                        ROS_FATAL("could not get any config file, i give up!");
                }
                nh->setParam("brainDiceConfigFile",brainDiceConfigFile.toStdString());
        }else{
                brainDiceConfigFile = QString::fromStdString(configFile);
        }

        ui.brainDiceConfigFile->setText(brainDiceConfigFile);
        dicingConfigUpdate();
        buttonStateUpdate();

        ui.position->addGraph();
        ui.position->graph(0)->setPen(QColor(Qt::red));
        ui.position->addGraph();
        ui.position->graph(1)->setPen(QColor(Qt::green));
        ui.position->addGraph();
        ui.position->graph(2)->setPen(QColor(Qt::blue));

        ui.velocity->addGraph();
        ui.velocity->graph(0)->setPen(QColor(Qt::red));
        ui.velocity->addGraph();
        ui.velocity->graph(1)->setPen(QColor(Qt::green));
        ui.velocity->addGraph();
        ui.velocity->graph(2)->setPen(QColor(Qt::blue));

        QObject::connect(ui.calibrate_camera, SIGNAL(clicked()), this, SLOT(calibrateCamera()));
        QObject::connect(ui.calibrate_cleanser, SIGNAL(clicked()), this, SLOT(calibrateCleanser()));
        QObject::connect(ui.calibrate_96well_0, SIGNAL(clicked()), this, SLOT(calibrate96well_0()));
        QObject::connect(ui.calibrate_96well_1, SIGNAL(clicked()), this, SLOT(calibrate96well_1()));
        QObject::connect(ui.calibrate_brain, SIGNAL(clicked()), this, SLOT(calibrateBrain()));
        QObject::connect(ui.scan, SIGNAL(pressed()), this, SLOT(scan()));
        QObject::connect(ui.dry_run, SIGNAL(pressed()), this, SLOT(dry_run()));
        QObject::connect(ui.run, SIGNAL(pressed()), this, SLOT(run()));
        QObject::connect(ui.pause, SIGNAL(pressed()), this, SLOT(pause()));
        QObject::connect(ui.stop, SIGNAL(pressed()), this, SLOT(stop()));
        QObject::connect(ui.prev_slice, SIGNAL(pressed()), this, SLOT(prev_slice()));
        QObject::connect(ui.next_slice, SIGNAL(pressed()), this, SLOT(next_slice()));
        QObject::connect(ui.prev_cube, SIGNAL(pressed()), this, SLOT(prev_cube()));
        QObject::connect(ui.next_cube, SIGNAL(pressed()), this, SLOT(next_cube()));
        QObject::connect(ui.scan_96well, SIGNAL(pressed()), this, SLOT(scan_96well()));
        QObject::connect(ui.swap_96well, SIGNAL(pressed()), this, SLOT(swap_96well()));
        QObject::connect(ui.auto_focus, SIGNAL(clicked()), this, SLOT(auto_focus()));
        QObject::connect(ui.lights, SIGNAL(clicked()), this, SLOT(lights()));
        QObject::connect(ui.clean, SIGNAL(clicked()), this, SLOT(clean()));

        QObject::connect(ui.x_plus, SIGNAL(pressed()), this, SLOT(move()));
        QObject::connect(ui.x_minus, SIGNAL(pressed()), this, SLOT(move()));
        QObject::connect(ui.y_plus, SIGNAL(pressed()), this, SLOT(move()));
        QObject::connect(ui.y_minus, SIGNAL(pressed()), this, SLOT(move()));
        QObject::connect(ui.z_plus, SIGNAL(pressed()), this, SLOT(move()));
        QObject::connect(ui.z_minus, SIGNAL(pressed()), this, SLOT(move()));
        QObject::connect(ui.z_minus, SIGNAL(pressed()), this, SLOT(move()));
        QObject::connect(ui.plan, SIGNAL(clicked()), this, SLOT(planUpdate()));
        QObject::connect(this, SIGNAL(updatePlan(int)), this, SLOT(planUpdate(int)));

        QObject::connect(ui.zero, SIGNAL(pressed()), this, SLOT(zero()));

        QObject::connect(this, SIGNAL(updateMotorState()), this, SLOT(plotData()));
        QObject::connect(this, SIGNAL(scanAreaUpdate()), this, SLOT(drawScanArea()));
        QObject::connect(this, SIGNAL(HUDupdate(int)), this, SLOT(drawHUD(int)));
        QObject::connect(this, SIGNAL(updateButtonStates()), this, SLOT(buttonStateUpdate()));
        QObject::connect(this, SIGNAL(updateDicingConfig()), this, SLOT(dicingConfigUpdate()));
        QObject::connect(ui.auto_plan, SIGNAL(clicked()), this, SLOT(autoPlan()));
        QObject::connect(ui.auto_plan_threshold, SIGNAL(sliderReleased()), this, SLOT(autoPlan()));

        ui.cnc_area->installEventFilter( this );
        ui.camera->installEventFilter( this );
        ui.plan_area->installEventFilter( this );

        start_time = ros::Time::now();
        messages_received["motor_state"] = 0;

        status_subscriber = nh->subscribe("/motor/state",1,&CNCGUI::StatusReceiver,this);
        motor_command = nh->advertise<geometry_msgs::Vector3>("/motor/command",1);
        neopixel_all_pub = nh->advertise<std_msgs::ColorRGBA>("/neopixel/all",1);
        cleanser_pub = nh->advertise<std_msgs::Int32>("/cleanser",1);

        initCamera(camera_calibration_file_path_lo_res,camera_calibration_file_path_hi_res,camera_id);

        frame.resize(2);
        frame_reticle.resize(2);
        cnc_area_image.resize(2);
        scan_area_image.resize(2);
        plan_image.resize(2);
        plan_image_reticle.resize(2);
        cnc_area_image_reticle.resize(2);
        cnc_area_image[LOW_RES] = cv::Mat(cv::Size(int((cnc_x_dim*pixel_per_mm_x[LOW_RES])+lo_res.height),
                                                   int((cnc_y_dim*pixel_per_mm_y[LOW_RES])+lo_res.width)),CV_8UC3);
        cnc_area_image[LOW_RES].setTo(Scalar(255,255,255));
        ROS_INFO("cnc area (%.1f %.1f)mm with image size (%d %d)pixel",
                 cnc_x_dim, cnc_y_dim, cnc_area_image[LOW_RES].size().height, cnc_area_image[LOW_RES].size().width);

        spinner.reset(new ros::AsyncSpinner(0));
        spinner->start();

        zero_srv = nh->serviceClient<std_srvs::Trigger>("/zero");

        ROS_INFO("cnc gui initialized");
}

void CNCGUI::shutdownPlugin() {
        // turn off the lights
        std_msgs::ColorRGBA msg;
        neopixel_all_pub.publish(msg);
        // shutdown communication
        ros::shutdown();
        if(frame_grabber_thread) {
                if(frame_grabber_thread->joinable()) {
                        frame_grabber_thread->join();
                }
        }
        if(scan_thread) {
                if(scan_thread->joinable()) {
                        scan_thread->join();
                }
        }
        if(qr_code_tracking_thread) {
                if(qr_code_tracking_thread->joinable()) {
                        qr_code_tracking_thread->join();
                }
        }
        if(qr_code_detection_thread) {
                if(qr_code_detection_thread->joinable()) {
                        qr_code_detection_thread->join();
                }
        }
        if(auto_focus_monitoring_thread) {
                if(auto_focus_monitoring_thread->joinable()) {
                        auto_focus_monitoring_thread->join();
                }
        }
        if(calibrate_camera_thread) {
                if(calibrate_camera_thread->joinable()) {
                        calibrate_camera_thread->join();
                }
        }
        if(calibrate_cleanser_thread) {
                if(calibrate_cleanser_thread->joinable()) {
                        calibrate_cleanser_thread->join();
                }
        }
        if(calibrate_96well_thread) {
                if(calibrate_96well_thread->joinable()) {
                        calibrate_96well_thread->join();
                }
        }
        if(calibrate_brain_thread) {
                if(calibrate_brain_thread->joinable()) {
                        calibrate_brain_thread->join();
                }
        }
        if(plan_dicing_thread) {
                if(plan_dicing_thread->joinable()) {
                        plan_dicing_thread->join();
                }
        }
        if(joystick_control_thread) {
                if(joystick_control_thread->joinable()) {
                        joystick_control_thread->join();
                }
        }
}

void CNCGUI::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                          qt_gui_cpp::Settings &instance_settings) const {
}

void CNCGUI::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                             const qt_gui_cpp::Settings &instance_settings) {

        frame_grabber_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::FrameGrabberThread, this));
        frame_grabber_thread->detach();
        auto_focus_monitoring_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::AutoFocusMonitorThread, this));
        auto_focus_monitoring_thread->detach();
        lights();
        loadPlanImage();
        Q_EMIT updatePlan(false);
        calculate96wellPositions();
        joystick_control_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::JoyStickContolThread, this));
        joystick_control_thread->detach();
}

void CNCGUI::CubeShot(int type){
        Mat img;
        if(grabFrame(img,HIGH_RES)) {
                cv::Rect myROI(hi_res.height/2-tool_size/2*pixel_per_mm_x[HIGH_RES],
                               hi_res.width/2-tool_size/2*pixel_per_mm_y[HIGH_RES],
                               int(tool_size*pixel_per_mm_x[HIGH_RES]),
                               int(tool_size*pixel_per_mm_y[HIGH_RES]));
                Mat cube_view = img(myROI);
                switch (type) {
                case 0: // pre dice
                        drawImage(cube_view,ui.cube_view_pre_dice);
                        break;
                case 1: // post dice
                        drawImage(cube_view,ui.cube_view_post_dice);
                        break;
                case 2: // pre dispense
                        drawImage(cube_view,ui.cube_view_pre_dispense);
                        break;
                case 3: // post dispense
                        drawImage(cube_view,ui.cube_view_post_dispense);
                        break;
                }
        }
}

bool CNCGUI::eventFilter( QObject* watched, QEvent* event ) {
        if ( watched == ui.cnc_area ) {
                switch (event->type()) {
                case QEvent::MouseButtonPress: {
                        const QMouseEvent* const me = static_cast<const QMouseEvent*>( event );
                        const QPoint p = me->pos(); //...or ->globalPos();
                        switch (me->button()) {
                        case Qt::LeftButton: {
                                geometry_msgs::Vector3 msg;
                                msg.x = (p.x()*cnc_area_image[LOW_RES].cols/ui.cnc_area->width()-lo_res.height/2)/pixel_per_mm_x[LOW_RES];
                                msg.y = cnc_y_dim-(p.y()*cnc_area_image[LOW_RES].rows/ui.cnc_area->height()-lo_res.width/2)/pixel_per_mm_y[LOW_RES];
                                motor_command.publish(msg);
                                break;
                        }
                        case Qt::MiddleButton: {
                                geometry_msgs::Vector3 msg;
                                msg.x = (p.x()*cnc_area_image[LOW_RES].cols/ui.cnc_area->width()-lo_res.height/2)/pixel_per_mm_x[LOW_RES]+tool_camera_offset_x;
                                msg.y = cnc_y_dim-(p.y()*cnc_area_image[LOW_RES].rows/ui.cnc_area->height()-lo_res.width/2)/pixel_per_mm_y[LOW_RES]+tool_camera_offset_y;
                                motor_command.publish(msg);
                                break;
                        }
                        default: return false;
                        }
                }
                case QEvent::MouseMove: {
                        const QMouseEvent* const me = static_cast<const QMouseEvent*>( event );
                        mouse_cursor_cnc_area = me->pos(); //...or ->globalPos();
                        break;
                }
                }
        }else if ( watched == ui.camera ) {
                switch (event->type()) {
                case QEvent::MouseButtonPress: {
                        const QMouseEvent* const me = static_cast<const QMouseEvent*>( event );
                        const QPoint p = me->pos(); //...or ->globalPos();
                        switch (me->button()) {
                        case Qt::LeftButton: {
                                geometry_msgs::Vector3 msg;
                                msg.x = values["pos_x"].back()+(p.x()-ui.camera->width()/2)*(frame[LOW_RES].cols/ui.camera->width())/pixel_per_mm_x[LOW_RES];
                                msg.y = values["pos_y"].back()-(p.y()-ui.camera->height()/2)*(frame[LOW_RES].rows/ui.camera->height())/pixel_per_mm_y[LOW_RES];
                                motor_command.publish(msg);
                                break;
                        }
                        case Qt::RightButton: {
                                ui.confirm->setChecked(true);
                                break;
                        }
                        case Qt::MiddleButton: {
                                geometry_msgs::Vector3 msg;
                                msg.x = values["pos_x"].back()+(p.x()-ui.camera->width()/2)*(frame[LOW_RES].cols/ui.camera->width())/pixel_per_mm_x[LOW_RES];
                                msg.y = values["pos_y"].back()-(p.y()-ui.camera->height()/2)*(frame[LOW_RES].rows/ui.camera->height())/pixel_per_mm_y[LOW_RES];
                                if(!tool_toggle) {
                                        msg.x += tool_camera_offset_x;
                                        msg.y += tool_camera_offset_y;
                                        tool_toggle = true;
                                }else{
                                        msg.x -= tool_camera_offset_x;
                                        msg.y -= tool_camera_offset_y;
                                        tool_toggle = false;
                                }
                                motor_command.publish(msg);

                                break;
                        }
                        default: return false;
                        }
                }
                case QEvent::MouseMove: {
                        const QMouseEvent* const me = static_cast<const QMouseEvent*>( event );
                        mouse_cursor_camera.setX(me->pos().x() *frame[LOW_RES].cols/ui.camera->width());
                        mouse_cursor_camera.setY(me->pos().y() *frame[LOW_RES].rows/ui.camera->height());
                        break;
                }
                }
        }
        if ( watched == ui.plan_area ) {
                switch (event->type()) {
                case QEvent::MouseButtonPress: {
                        const QMouseEvent* const me = static_cast<const QMouseEvent*>( event );
                        const QPoint p = me->pos(); //...or ->globalPos();
                        switch (me->button()) {
                        case Qt::LeftButton: {
                                if(plan_operation==0)
                                        plan_operation = 1;
                                else
                                        plan_operation = 0;
                                // ROS_INFO("plan operation %d",plan_operation);
                                break;
                        }
                        case Qt::RightButton: {
                                if(plan_operation==0)
                                        plan_operation = 2;
                                else
                                        plan_operation = 0;
                                // ROS_INFO("plan operation %d",plan_operation);
                                break;
                        }
                        default: return false;
                        }
                }
                case QEvent::MouseMove: {
                        const QMouseEvent* const me = static_cast<const QMouseEvent*>( event );
                        float pos_x_sample =  me->pos().x()*plan_image_reticle[HIGH_RES].cols/ui.plan_area->width()/pixel_per_mm_x[HIGH_RES];
                        float pos_y_sample =  me->pos().y()*plan_image_reticle[HIGH_RES].rows/ui.plan_area->height()/pixel_per_mm_y[HIGH_RES];
                        mouse_cursor_plan_area.setX(me->pos().x());
                        mouse_cursor_plan_area.setY(me->pos().y());
                        int cubes_col = (pos_x_sample - (hi_res.height/2/pixel_per_mm_x[HIGH_RES]))/(tool_size+tool_offset);
                        int cubes_row = (pos_y_sample - (hi_res.width/2/pixel_per_mm_y[HIGH_RES]))/(tool_size+tool_offset);
                        // ROS_INFO_THROTTLE(1,"x: %.1f y: %.1f row: %d col: %d",pos_x_sample,pos_y_sample, cubes_row, cubes_col);
                        cube_hovered = cubes_col*cubes_dim.height+cubes_row;
                        if(cube_hovered>=0 && cube_hovered<cubes.size()) {
                                if(plan_operation==1) {
                                        cube_active[cube_hovered] = true;
                                }else if(plan_operation==2) {
                                        cube_active[cube_hovered] = false;
                                }
                        }else{
                                cube_hovered = 0;
                        }
                        drawPlan();
                        break;
                }
                }
        }
        return false;
}

void CNCGUI::calibrateCamera(){
        if(calibrate_camera_thread) // maybe it's running already, wait until done...
                if(calibrate_camera_thread->joinable())
                        calibrate_camera_thread->join();
        calibrate_camera_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::CalibrateCameraThread, this));
        calibrate_camera_thread->detach();
}

void CNCGUI::calibrateCleanser(){
        if(calibrate_cleanser_thread) // maybe it's running already, wait until done...
                if(calibrate_cleanser_thread->joinable())
                        calibrate_cleanser_thread->join();
        calibrate_cleanser_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::CalibrateCleanserThread, this));
        calibrate_cleanser_thread->detach();
}

void CNCGUI::calibrate96well_0(){
        if(calibrate_96well_thread) // maybe it's running already, wait until done...
                if(calibrate_96well_thread->joinable())
                        calibrate_96well_thread->join();
        calibrate_96well_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::Calibrate96wellThread, this, 0));
        calibrate_96well_thread->detach();
}

void CNCGUI::calibrate96well_1(){
        if(calibrate_96well_thread) // maybe it's running already, wait until done...
                if(calibrate_96well_thread->joinable())
                        calibrate_96well_thread->join();
        calibrate_96well_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::Calibrate96wellThread, this, 1));
        calibrate_96well_thread->detach();
}

void CNCGUI::calibrateBrain(){
        if(calibrate_brain_thread) // maybe it's running already, wait until done...
                if(calibrate_brain_thread->joinable())
                        calibrate_brain_thread->join();
        calibrate_brain_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::CalibrateBrainThread, this));
        calibrate_brain_thread->detach();
}

void CNCGUI::CalibrateCameraThread(){
        ROS_INFO("camera calibration thread START");
        ROS_INFO("moveing to saved optical reference %.1f %.1f",
                 optical_reference_pos.x, optical_reference_pos.y);
        WaitForPositionReachedSave(optical_reference_pos,0.01,1);
        if(!checkConfirm())
                return;
        ui.auto_focus->setChecked(true);
        ros::Time t0 = ros::Time::now(), t1;
        ROS_INFO("waiting for the camera to focus and the qr code to be centered");
        grab_frames = false;
        do {
                if((ros::Time::now()-t0).toSec()>30) {
                        ROS_WARN("timeout for qr_code tracking");
                        return;
                }
                if(!grabFrame(frame[LOW_RES], LOW_RES))
                        continue;
                qr_code_detected = detectQRCode(frame[LOW_RES],qr_code_corners,qr_code_pos,qr_code_rot,qr_code_size);
                if(qr_code_detected)
                        qr_tracking_error = trackQRCode();
                else
                        ROS_WARN_THROTTLE(1,"no qr code detected");
                drawHUD(frame[LOW_RES], frame_reticle[LOW_RES]);
                ROS_INFO_THROTTLE(1,"qr_tracking_error %.3f",qr_tracking_error);
        } while(qr_tracking_error>3);
        ROS_INFO("qr_tracking_error %.3f, starting calibration",qr_tracking_error);
        ros::Duration d(3-(ros::Time::now()-t0).toSec());
        d.sleep();
        qr_code_detected = false;
        focus_absolute = getV4L2("focus_absolute");
        optical_reference_pos.x = values["pos_x"].back();
        optical_reference_pos.y = values["pos_y"].back();
        ui.auto_focus->setChecked(false);

        vector<float> qr_code_side_length_px_x = {0,0},
                      qr_code_side_length_px_y = {0,0};
        vector<int> samples = {0,0};
        geometry_msgs::Vector3 msg;
        msg.x = optical_reference_pos.x;
        msg.y = optical_reference_pos.y;
        for(float y_offset = -3; y_offset<=3; y_offset+=3) {
                msg.y = optical_reference_pos.y+y_offset;
                for(float x_offset = -3; x_offset<=3; x_offset+=3) {
                        msg.x = optical_reference_pos.x+x_offset;
                        WaitForPositionReached(msg,0.1,3);
                        t1 = ros::Time::now();
                        while(!grabFrame(frame[LOW_RES], LOW_RES)) {
                                ROS_INFO_THROTTLE(1,"grabbing low res camera frame");
                                if((ros::Time::now()-t1).toSec()>5) {
                                        ROS_ERROR("grabbing camera frame timed out, aborting calibration, check thy camera");
                                        ui.auto_focus->setChecked(true);
                                        grab_frames = true;
                                        return;
                                }
                        }
                        bool detected = detectQRCode(frame[LOW_RES],qr_code_corners,qr_code_pos,qr_code_rot,qr_code_size);
                        if(detected) {
                                qr_code_side_length_px_y[LOW_RES] += sqrtf(powf(qr_code_corners[1].get_u()-qr_code_corners[0].get_u(),2.0f)+powf(qr_code_corners[1].get_v()-qr_code_corners[0].get_v(),2.0f));
                                qr_code_side_length_px_x[LOW_RES] += sqrtf(powf(qr_code_corners[2].get_u()-qr_code_corners[1].get_u(),2.0f)+powf(qr_code_corners[2].get_v()-qr_code_corners[1].get_v(),2.0f));
                                qr_code_side_length_px_y[LOW_RES] += sqrtf(powf(qr_code_corners[3].get_u()-qr_code_corners[2].get_u(),2.0f)+powf(qr_code_corners[3].get_v()-qr_code_corners[2].get_v(),2.0f));
                                qr_code_side_length_px_x[LOW_RES] += sqrtf(powf(qr_code_corners[0].get_u()-qr_code_corners[3].get_u(),2.0f)+powf(qr_code_corners[0].get_v()-qr_code_corners[3].get_v(),2.0f));
                                samples[LOW_RES]+=1;
                        }
                        t1 = ros::Time::now();
                        while(!grabFrame(frame[HIGH_RES], HIGH_RES)) {
                                ROS_INFO_THROTTLE(1,"grabbing high res camera frame");
                                if((ros::Time::now()-t1).toSec()>5) {
                                        ROS_ERROR("grabbing camera frame timed out, aborting calibration, check thy camera");
                                        ui.auto_focus->setChecked(true);
                                        grab_frames = true;
                                        return;
                                }
                        }
                        detected = detectQRCode(frame[HIGH_RES],qr_code_corners,qr_code_pos,qr_code_rot,qr_code_size);
                        if(detected) {
                                qr_code_side_length_px_y[HIGH_RES] += sqrtf(powf(qr_code_corners[1].get_u()-qr_code_corners[0].get_u(),2.0f)+powf(qr_code_corners[1].get_v()-qr_code_corners[0].get_v(),2.0f));
                                qr_code_side_length_px_x[HIGH_RES] += sqrtf(powf(qr_code_corners[2].get_u()-qr_code_corners[1].get_u(),2.0f)+powf(qr_code_corners[2].get_v()-qr_code_corners[1].get_v(),2.0f));
                                qr_code_side_length_px_y[HIGH_RES] += sqrtf(powf(qr_code_corners[3].get_u()-qr_code_corners[2].get_u(),2.0f)+powf(qr_code_corners[3].get_v()-qr_code_corners[2].get_v(),2.0f));
                                qr_code_side_length_px_x[HIGH_RES] += sqrtf(powf(qr_code_corners[0].get_u()-qr_code_corners[3].get_u(),2.0f)+powf(qr_code_corners[0].get_v()-qr_code_corners[3].get_v(),2.0f));
                                samples[HIGH_RES]+=1;
                        }
                        ROS_INFO("\nsample low res %d pixel_per_mm_x %f pixel_per_mm_y %f\n"
                                 "sample high res %d pixel_per_mm_x %f pixel_per_mm_y %f\n",
                                 samples[LOW_RES], qr_code_side_length_px_x[LOW_RES]/(2*samples[LOW_RES])/(qr_code_size-11),
                                 qr_code_side_length_px_y[LOW_RES]/(2*samples[LOW_RES])/(qr_code_size-11),
                                 samples[HIGH_RES], qr_code_side_length_px_x[HIGH_RES]/(2*samples[HIGH_RES])/(qr_code_size-11),
                                 qr_code_side_length_px_y[HIGH_RES]/(2*samples[HIGH_RES])/(qr_code_size-11)
                                 );
                }
        }
        float pixel_per_mm_x_low_res_temp = qr_code_side_length_px_x[LOW_RES]/(2*samples[LOW_RES])/(qr_code_size-11); // the actual qr_code corners a inset by 11mm
        float pixel_per_mm_y_low_res_temp = qr_code_side_length_px_y[LOW_RES]/(2*samples[LOW_RES])/(qr_code_size-11); // the actual qr_code corners a inset by 11mm
        float pixel_per_mm_x_high_res_temp = qr_code_side_length_px_x[HIGH_RES]/(2*samples[HIGH_RES])/(qr_code_size-11); // the actual qr_code corners a inset by 11mm
        float pixel_per_mm_y_high_res_temp = qr_code_side_length_px_y[HIGH_RES]/(2*samples[HIGH_RES])/(qr_code_size-11); // the actual qr_code corners a inset by 11mm
        if(!isnan(pixel_per_mm_y_high_res_temp)) {
                pixel_per_mm_x[LOW_RES] = pixel_per_mm_x_low_res_temp;
                pixel_per_mm_y [LOW_RES]= pixel_per_mm_y_low_res_temp;
                pixel_per_mm_x[HIGH_RES] = pixel_per_mm_x_high_res_temp;
                pixel_per_mm_y [HIGH_RES] = pixel_per_mm_y_high_res_temp;
        }else{
                ROS_ERROR("pixel width was nan, aborting calibration");
                grab_frames = true;
                return;
        }
        cnc_area_image[LOW_RES] = cv::Mat(cv::Size(int((cnc_x_dim*pixel_per_mm_x[LOW_RES])+lo_res.height),
                                                   int((cnc_y_dim*pixel_per_mm_y[LOW_RES])+lo_res.width)),CV_8UC3);
        cnc_area_image[HIGH_RES] = cv::Mat(cv::Size(int((cnc_x_dim*pixel_per_mm_x[HIGH_RES])+hi_res.height),
                                                    int((cnc_y_dim*pixel_per_mm_y[HIGH_RES])+hi_res.width)),CV_8UC3);
        ROS_INFO("low resolution scan area (%.1f %.1f)mm with image size (%d %d)pixel",
                 cnc_x_dim, cnc_y_dim, cnc_area_image[LOW_RES].size().height, cnc_area_image[LOW_RES].size().width);
        ROS_INFO("high resolution scan area (%.1f %.1f)mm with image size (%d %d)pixel",
                 cnc_x_dim, cnc_y_dim, cnc_area_image[HIGH_RES].size().height, cnc_area_image[HIGH_RES].size().width);
        Q_EMIT updateDicingConfig();
        msg.x = optical_reference_pos.x;
        msg.y = optical_reference_pos.y;
        WaitForPositionReached(msg,0.1,3);
        grab_frames = true;
        ROS_INFO("camera calibration thread STOP");
}

void CNCGUI::CalibrateCleanserThread(){
        ROS_INFO("cleanser calibration START");
        ROS_INFO("moveing to saved cleanser position %.1f %.1f %.1f",
                 cleanser_pos.x, cleanser_pos.y, cleanser_pos.z);
        WaitForPositionReachedSave(cleanser_pos,0.1,120);
        if(!checkConfirm())
                return;

        cleanser_pos.x = values["pos_x"].back();
        cleanser_pos.y = values["pos_y"].back();
        cleanser_pos.z = values["pos_z"].back();
        Q_EMIT updateDicingConfig();
        ROS_INFO("manually move the tool into the cleanser to the desired depth");
        MoveToolSave(cleanser_pos.z-cleanser_tool_depth);
        if(!checkConfirm())
                return;
        cleanser_tool_depth = cleanser_pos.z-values["pos_z"].back();
        Q_EMIT updateDicingConfig();
        MoveToolSave(0);
        ROS_INFO("cleanser calibration STOP");
}

void CNCGUI::Calibrate96wellThread(int number){
        ROS_INFO("96well calibration START");
        ROS_INFO("moveing to saved top left 96well %d position %.1f %.1f", number,
                 ninety_six_well_top_left[number].x,
                 ninety_six_well_top_left[number].y);
        geometry_msgs::Vector3 msg = ninety_six_well_top_left[number];
        msg.z = 0;
        WaitForPositionReachedSave(msg,0.01,1);
        if(!checkConfirm(120)) // 2 minutes timeout
                return;
        ninety_six_well_top_left[number].x = values["pos_x"].back();
        ninety_six_well_top_left[number].y = values["pos_y"].back();
        msg = ninety_six_well_top_left[number];
        msg.x+=tool_camera_offset_x;
        msg.y+=tool_camera_offset_y;
        msg.z=0;
        WaitForPositionReachedSave(msg,0.1,10);
        MoveTool(ninety_six_well_top_left[number].z);
        ROS_INFO("move blade manually until it just about touches the 96 well");
        if(!checkConfirm(120)) // 2 minutes timeout
                return;
        ninety_six_well_top_left[number].z = values["pos_z"].back();
        Q_EMIT updateDicingConfig();

        //////////////////
        ROS_INFO("moveing to saved bottom right 96well %d position %.1f %.1f", number,
                 ninety_six_well_bottom_right[number].x,
                 ninety_six_well_bottom_right[number].y);
        msg = ninety_six_well_bottom_right[number];
        msg.z = 0;
        WaitForPositionReachedSave(msg,0.01,1);
        if(!checkConfirm(120)) // 2 minutes timeout
                return;
        ninety_six_well_bottom_right[number].x = values["pos_x"].back();
        ninety_six_well_bottom_right[number].y = values["pos_y"].back();
        ROS_INFO("move blade manually until it just about touches the 96 well");
        msg = ninety_six_well_bottom_right[number];
        msg.x+=tool_camera_offset_x;
        msg.y+=tool_camera_offset_y;
        msg.z=0;
        WaitForPositionReachedSave(msg,0.1,10);
        MoveTool(ninety_six_well_bottom_right[number].z);
        ROS_INFO("move blade manually until it just about touches the 96 well");
        if(!checkConfirm(120)) // 2 minutes timeout
                return;
        ninety_six_well_bottom_right[number].z = values["pos_z"].back();
        Q_EMIT updateDicingConfig();
        MoveTool(0);
        calculate96wellPositions();
        //////////////
        ROS_INFO("96well calibration STOP");
}

void CNCGUI::CalibrateBrainThread(){
        ROS_INFO("brain calibration START");
        ROS_INFO("moveing to saved top left brain sample position %.1f %.1f",
                 brain_sample_top_left.x,
                 brain_sample_top_left.y);
        geometry_msgs::Vector3 msg = brain_sample_top_left;
        msg.z = 0;
        WaitForPositionReachedSave(msg,0.01,1);
        if(!checkConfirm(120)) // 2 minutes timeout
                return;
        brain_sample_top_left.x = values["pos_x"].back();
        brain_sample_top_left.y = values["pos_y"].back();
        msg = brain_sample_top_left;
        msg.x+=tool_camera_offset_x;
        msg.y+=tool_camera_offset_y;
        msg.z=0;
        WaitForPositionReachedSave(msg,0.1,10);
        MoveTool(brain_sample_top_left.z);
        ROS_INFO("move blade manually until it just about touches the sample");
        if(!checkConfirm(120)) // 2 minutes timeout
                return;
        brain_sample_top_left.z = values["pos_z"].back();
        Q_EMIT updateDicingConfig();

        //////////////////
        ROS_INFO("moveing to saved bottom right brain sample position %.1f %.1f",
                 brain_sample_bottom_right.x,
                 brain_sample_bottom_right.y);
        msg = brain_sample_bottom_right;
        msg.z = 0;
        WaitForPositionReachedSave(msg,0.01,1);
        if(!checkConfirm(120)) // 2 minutes timeout
                return;
        brain_sample_bottom_right.x = values["pos_x"].back();
        brain_sample_bottom_right.y = values["pos_y"].back();
        ROS_INFO("move blade manually until it just about touches the 96 well");
        msg = brain_sample_bottom_right;
        msg.x+=tool_camera_offset_x;
        msg.y+=tool_camera_offset_y;
        msg.z=0;
        WaitForPositionReachedSave(msg,0.1,10);
        MoveTool(brain_sample_bottom_right.z);
        ROS_INFO("move blade manually until it just about touches the 96 well");
        if(!checkConfirm(120)) // 2 minutes timeout
                return;
        brain_sample_bottom_right.z = values["pos_z"].back();
        Q_EMIT updateDicingConfig();
        MoveTool(0);
        ROS_INFO("brain calibration STOP");
}

void CNCGUI::DryRunThread(){
        ROS_INFO("dry run START");
        ui.pause->setEnabled(true);
        ui.stop->setEnabled(true);
        well_counter = 0;
        bool abort= false;
        for(int i=0; i<ninety_six_well_content.size(); i++) {
                for(int j=0; j<ninety_six_well_content[i].size(); j++) {
                        geometry_msgs::Vector3 msg = brain_sample_top_left;
                        cube_target = ninety_six_well_content[i][j];
                        //// sample camera position check
                        auto pos = cubes[cube_target];
                        msg.x+=(pos.x-(hi_res.height/2/pixel_per_mm_x[HIGH_RES])+tool_size/2);
                        msg.y+=(-pos.y+(hi_res.width/2/pixel_per_mm_y[HIGH_RES])-tool_size/2);
                        msg.z = -1;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        ROS_INFO("cube %d of ninety_six_well %ld at camera position %.1f %.1f",
                                 cube_target,ninety_six_well_IDs[i],pos.x,pos.y);
                        drawPlan();
                        CubeShot(PRE_DICE);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }
                        //// sample tool position check
                        msg.x += tool_camera_offset_x;
                        msg.y += tool_camera_offset_y;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        MoveTool(brain_sample_top_left.z);
                        ROS_INFO("cube %d of ninety_six_well %ld at tool position %.1f %.1f",
                                 cube_target,ninety_six_well_IDs[i],pos.x,pos.y);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }
                        //// Post dice check
                        msg = brain_sample_top_left;
                        msg.x+=(pos.x-(hi_res.height/2/pixel_per_mm_x[HIGH_RES])+tool_size/2);
                        msg.y+=(-pos.y+(hi_res.width/2/pixel_per_mm_y[HIGH_RES])-tool_size/2);
                        msg.z = -1;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        ROS_INFO("cube %d of ninety_six_well %ld at camera position %.1f %.1f",
                                 cube_target,ninety_six_well_IDs[i],pos.x,pos.y);
                        CubeShot(POST_DICE);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }
                        //// 96well camera position check
                        msg.x = ninety_six_well_pos[well_counter].x;
                        msg.y = ninety_six_well_pos[well_counter].y;
                        msg.z = -1;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        ROS_INFO("well %d at camera position %.1f %.1f",well_counter,msg.x,msg.y);
                        CubeShot(PRE_DISPENSE);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }
                        //// 96well tool position check
                        msg.x += tool_camera_offset_x;
                        msg.y += tool_camera_offset_y;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        MoveTool(ninety_six_well_top_left[0].z);
                        ROS_INFO("well %d at tool position %.1f %.1f",well_counter,msg.x,msg.y);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }
                        //// 96well camera post dispense check
                        msg.x = ninety_six_well_pos[well_counter].x;
                        msg.y = ninety_six_well_pos[well_counter].y;
                        msg.z = -1;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        ROS_INFO("well %d at camera position %.1f %.1f",well_counter,msg.x,msg.y);
                        CubeShot(POST_DISPENSE);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }

                        well_counter++;
                        if(well_counter==96) {
                                ROS_INFO("first 96well full");
                        }else if(well_counter==96*2) {
                                ROS_INFO("second 96well full, now would be a good time to change them");
                                Clean();
                                well_counter=0;
                                QMessageBox msgBox;
                                msgBox.setText("96 wells are full");
                                msgBox.setInformativeText("Hit OK to continue, once you replaced them");
                                msgBox.setStandardButtons(QMessageBox::Ok |QMessageBox::Cancel);
                                msgBox.setDefaultButton(QMessageBox::Cancel);
                                int ret = msgBox.exec();
                                if(ret==QMessageBox::Ok) {
                                        ROS_INFO("resuming...");
                                }else{
                                        ROS_INFO("aborting...");
                                        abort = true;
                                        break;
                                }
                        }
                        if(well_counter%cleanser_frequency==0) {
                                Clean();
                        }
                }
                if(abort) {
                        break;
                }
        }
        if(abort) {
                ui.pause->setEnabled(false);
                ui.stop->setEnabled(false);
        }
        MoveToolSave(0);
        ROS_INFO("dry run STOP");
}

void CNCGUI::RunThread(){
        ROS_INFO("run START");
        bool abort = false;
        ui.pause->setEnabled(true);
        ui.stop->setEnabled(true);
        for(int i=cube/96; i<ninety_six_well_content.size(); i++) {
                for(int j=cube%96; j<ninety_six_well_content[i].size(); j++) {
                        geometry_msgs::Vector3 msg = brain_sample_top_left;
                        cube_target = ninety_six_well_content[i][j];
                        //// sample camera position check
                        auto pos = cubes[cube_target];
                        msg.x+=(pos.x-(hi_res.height/2/pixel_per_mm_x[HIGH_RES])+tool_size/2);
                        msg.y+=(-pos.y+(hi_res.width/2/pixel_per_mm_y[HIGH_RES])-tool_size/2);
                        msg.z = -1;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        ROS_INFO("cube %d of ninety_six_well %ld at camera position %.1f %.1f",
                                 cube_target,ninety_six_well_IDs[i],pos.x,pos.y);
                        drawPlan();
                        CubeShot(PRE_DICE);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }
                        //// sample tool position check
                        msg.x += tool_camera_offset_x;
                        msg.y += tool_camera_offset_y;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        MoveTool(brain_sample_top_left.z);
                        ROS_INFO("cube %d of ninety_six_well %ld at tool position %.1f %.1f",
                                 cube_target,ninety_six_well_IDs[i],pos.x,pos.y);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }
                        ROS_INFO("cutting to depth %.1f", cut_depth);
                        if(!MoveToolSave(brain_sample_top_left.z-cut_depth)) {
                                abort = true;
                                break;
                        }
                        ROS_INFO("returning to surface");
                        if(!MoveToolSave(brain_sample_top_left.z)) {
                                abort = true;
                                break;
                        }
                        ROS_INFO("dwelling on surface for %.1f seconds",dwell_on_surface);
                        ros::Duration dwell(dwell_on_surface);
                        dwell.sleep();
                        //// Post dice check
                        msg = brain_sample_top_left;
                        msg.x+=(pos.x-(hi_res.height/2/pixel_per_mm_x[HIGH_RES])+tool_size/2);
                        msg.y+=(-pos.y+(hi_res.width/2/pixel_per_mm_y[HIGH_RES])-tool_size/2);
                        msg.z = -1;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        ROS_INFO("cube %d of ninety_six_well %ld at camera position %.1f %.1f",
                                 cube_target,ninety_six_well_IDs[i],pos.x,pos.y);
                        CubeShot(POST_DICE);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }
                        //// 96well camera position check
                        msg.x = ninety_six_well_pos[ninety_six_well_counter].x;
                        msg.y = ninety_six_well_pos[ninety_six_well_counter].y;
                        msg.z = -1;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        ROS_INFO("well %d at camera position %.1f %.1f",
                                 ninety_six_well_counter,msg.x,msg.y);
                        CubeShot(PRE_DISPENSE);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }
                        //// dispense
                        ROS_INFO("moving to well position");
                        msg.x = ninety_six_well_pos[ninety_six_well_counter].x+tool_camera_offset_x;
                        msg.y = ninety_six_well_pos[ninety_six_well_counter].y+tool_camera_offset_y;
                        msg.z = -1;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        if(!MoveToolSave(ninety_six_well_top_left[0].z)) {
                                abort = true;
                                break;
                        }
                        if(!ui.confirmAll->isChecked()) {
                                ROS_INFO("hit confirm to perform dispense move");
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }
                        for(int rep=0; rep<dispense_repetitions; rep++) {
                                if(!MoveToolSave(ninety_six_well_top_left[0].z-dispense_depth)) {
                                        abort = true;
                                        break;
                                }
                                if(!MoveToolSave(ninety_six_well_top_left[0].z)) {
                                        abort = true;
                                        break;
                                }
                        }
                        //// 96well camera post dispense check
                        msg.x = ninety_six_well_pos[ninety_six_well_counter].x;
                        msg.y = ninety_six_well_pos[ninety_six_well_counter].y;
                        msg.z = -1;
                        if(!WaitForPositionReachedSave(msg,0.1,20)) {
                                ROS_ERROR("could not reach positions, aborting...");
                                abort = true;
                                break;
                        }
                        ROS_INFO("well %d at camera position %.1f %.1f",
                                 ninety_six_well_counter,msg.x,msg.y);
                        CubeShot(POST_DISPENSE);
                        if(!ui.confirmAll->isChecked()) {
                                if(!checkConfirm(60)) { // wait 60 seconds
                                        abort = true;
                                        break;
                                }
                        }

                        cube++;
                        ninety_six_well_counter++;
                        Q_EMIT updateDicingConfig();
                        Q_EMIT updateButtonStates();
                        if(ninety_six_well_counter==96) {
                                ROS_INFO("first 96well full");
                        }else if(ninety_six_well_counter==96*2) {
                                ROS_INFO("second 96well full, now would be a good time to change them");
                                Clean();
                                ninety_six_well_counter=0;
                                QMessageBox msgBox;
                                msgBox.setText("96 wells are full");
                                msgBox.setInformativeText("Hit OK to continue, once you replaced them");
                                msgBox.setStandardButtons(QMessageBox::Ok |QMessageBox::Cancel);
                                msgBox.setDefaultButton(QMessageBox::Cancel);
                                int ret = msgBox.exec();
                                if(ret==QMessageBox::Ok) {
                                        ROS_INFO("resuming...");
                                }else{
                                        ROS_INFO("aborting...");
                                        abort = true;
                                        break;
                                }
                        }
                        if(ninety_six_well_counter%cleanser_frequency==0) {
                                Clean();
                        }
                }
                if(abort) {
                        break;
                }
        }
        MoveToolSave(0);
        if(abort) {
                ui.pause->setEnabled(false);
                ui.stop->setEnabled(false);
        }
        ROS_INFO("run STOP");
}

bool CNCGUI::checkConfirm(int timeout_sec){
        ros::Time t0 = ros::Time::now(), t1 = ros::Time::now();
        ui.confirm->setChecked(false);
        ui.abort->setChecked(false);
        bool toggle = false;
        while(!ui.confirm->isChecked()) {
                ROS_INFO_THROTTLE(1,"waiting for confirm or abort");
                if(ui.abort->isChecked() || (ros::Time::now()-t0).toSec()>timeout_sec) {
                        ROS_WARN("abort");
                        ui.confirm->setChecked(false);
                        ui.abort->setChecked(false);
                        ui.LED->setStyleSheet("background-color: lightgray");
                        return false;
                }
                if((ros::Time::now()-t1).toSec()>1) {
                        toggle = !toggle;
                        t1 = ros::Time::now();
                        if(toggle) {
                                ui.LED->setStyleSheet("background-color: green");
                        }else{
                                ui.LED->setStyleSheet("background-color: red");
                        }
                }
        }
        ui.confirm->setChecked(false);
        ui.abort->setChecked(false);
        ui.LED->setStyleSheet("background: lightgray");
        return true;
}

bool CNCGUI::WaitForPositionReachedSave(geometry_msgs::Vector3 &msg, float error, float timeout){
        geometry_msgs::Vector3 msg_save;
        msg_save.x = values["pos_x"].back();
        msg_save.y = values["pos_y"].back();
        msg_save.z = -1;
        motor_command.publish(msg_save);
        ros::Time t0 = ros::Time::now();
        float cnc_error = 1000;
        while((ros::Time::now()-t0).toSec()<timeout && cnc_error>1) {
                cnc_error = sqrtf(powf(values["pos_x"].back()-msg_save.x,2.0f)+
                                  powf(values["pos_y"].back()-msg_save.y,2.0f)+
                                  powf(values["pos_z"].back()-msg_save.z,2.0f));
        }
        msg_save.x = msg.x;
        msg_save.y = msg.y;
        msg_save.z = -1;
        motor_command.publish(msg_save);
        cnc_error = 1000;
        while((ros::Time::now()-t0).toSec()<timeout && cnc_error>1) {
                cnc_error = sqrtf(powf(values["pos_x"].back()-msg_save.x,2.0f)+
                                  powf(values["pos_y"].back()-msg_save.y,2.0f)+
                                  powf(values["pos_z"].back()-msg_save.z,2.0f));
        }
        motor_command.publish(msg);
        cnc_error = 1000;
        while((ros::Time::now()-t0).toSec()<timeout && cnc_error>error) {
                cnc_error = sqrtf(powf(values["pos_x"].back()-msg.x,2.0f)+
                                  powf(values["pos_y"].back()-msg.y,2.0f)+
                                  powf(values["pos_z"].back()-msg.z,2.0f));
        }
        if(cnc_error>error) {
                ROS_WARN("position %.3f %.3f %.3f timed out at cnc position %.3f %.3f %.3f",
                         msg.x,msg.y,msg.z,values["pos_x"].back(),values["pos_y"].back(),values["pos_z"].back());
                return false;
        }
        return true;
}

void CNCGUI::WaitForPositionReached(geometry_msgs::Vector3 &msg, float error, float timeout){
        motor_command.publish(msg);
        ros::Time t0 = ros::Time::now();
        float cnc_error = 1000;
        while((ros::Time::now()-t0).toSec()<timeout && cnc_error>error) {
                cnc_error = sqrtf(powf(values["pos_x"].back()-msg.x,2.0f)+powf(values["pos_y"].back()-msg.y,2.0f));
        }
        if(cnc_error>error)
                ROS_WARN("position %.3f %.3f timed out at cnc position %.3f %.3f",msg.x,msg.y,values["pos_x"].back(),values["pos_y"].back());
}

float CNCGUI::trackQRCode(float x, float y){
        geometry_msgs::Vector3 msg;
        float error_x = (x-qr_code_pos[0]);
        float error_y = (y-qr_code_pos[1]);
        if(fabsf(error_x)<1) {
                msg.x = values["pos_x"].back()-0.01*error_x;
        }else if(fabsf(error_x)<5) {
                msg.x = values["pos_x"].back()-0.1*error_x;
        }else{
                msg.x = values["pos_x"].back()-error_x;
        }
        if(fabsf(error_y)<1) {
                msg.y = values["pos_y"].back()+0.01*error_y;
        }else if(fabsf(error_y)<5) {
                msg.y = values["pos_y"].back()+0.1*error_y;
        }else{
                msg.y = values["pos_y"].back()+error_y;
        }

        motor_command.publish(msg);
        return sqrtf(powf(error_x,2.0)+powf(error_y,2.0));
}

void CNCGUI::drawLine(cv::Mat &img, int x0, int y0, int x1, int y1, cv::Scalar color, int thickness){
        cv::line(img,cv::Point2i(x0,y0),cv::Point2i(x1,y1),color, thickness);
}

string CNCGUI::exec(const char* cmd) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe) {
                throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
                result += buffer.data();
        }
        return result;
}

int CNCGUI::getV4L2(string value){
        char str[300];
        sprintf(str,"v4l2-ctl --device=%d -C %s",camera_id,value.c_str());
        string val = exec(str);
        string output = std::regex_replace(
                val,
                std::regex("[^0-9]*([0-9]+).*"),
                std::string("$1")
                );
        return std::stoi (output,nullptr,0);
}

void CNCGUI::setV4L2(string name, int value){
        char str[300];
        sprintf(str,"v4l2-ctl --device=%d -c %s=%d",camera_id,name.c_str(),value);
        exec(str);
}

void CNCGUI::AutoFocusMonitorThread(){
        ROS_INFO("starting auto focus monitoring thread");
        ros::Rate rate(1);
        while(ros::ok()) {
                if(ui.auto_focus->isChecked()) {
                        if(getV4L2("exposure_auto")!=3)
                                setV4L2("exposure_auto",3);
                        if(getV4L2("focus_auto")!=1)
                                setV4L2("focus_auto",1);
                        ui.focus_absolute->setText(QString::number(getV4L2("focus_absolute")));
                        ui.exposure_absolute->setText(QString::number(getV4L2("exposure_absolute")));
                }else{
                        // force manual focus and manual exposure
                        setV4L2("exposure_auto",1);
                        setV4L2("focus_auto",0);
                        setV4L2("focus_absolute",focus_absolute);
                        setV4L2("exposure_absolute",exposure_absolute);
                        ui.focus_absolute->setText(QString::number(focus_absolute));
                        ui.exposure_absolute->setText(QString::number(exposure_absolute));
                }
                rate.sleep();
        }
        ROS_INFO("stopping auto focus monitoring thread");
}

void CNCGUI::auto_focus(){
        if(ui.auto_focus->isChecked()) {
                setV4L2("focus_auto",1);
                setV4L2("exposure_auto",3);
        }else{
                ROS_INFO("manual focus");
                setV4L2("focus_auto",0);
                setV4L2("focus_absolute",focus_absolute);
                setV4L2("exposure_auto",1);
                setV4L2("exposure_absolute",exposure_absolute);
        }
}

void CNCGUI::lights(){
        std_msgs::ColorRGBA msg;
        if(ui.lights->isChecked()) {
                msg.r = 1;
                msg.g = 1;
                msg.b = 1;
        }
        neopixel_all_pub.publish(msg);
}

void CNCGUI::clean(){
        ui.clean->setEnabled(false);
        Clean();
        ui.clean->setEnabled(true);
        ui.clean->setChecked(false);
}

void CNCGUI::drawHUD(Mat &img_src, Mat &img_dst, int res){
        // reticle
        img_dst = img_src.clone();
        drawLine(img_dst,0,img_dst.rows/2,img_dst.cols,img_dst.rows/2,cv::Scalar(255,0,0),2);
        drawLine(img_dst,img_dst.cols/2,0,img_dst.cols/2,img_dst.rows,cv::Scalar(255,0,0),2);
        for(int d_horizontal = -50; d_horizontal<=50; d_horizontal+=10) {
                int y = img_dst.rows/2+int(d_horizontal*pixel_per_mm_y[res]);
                drawLine(img_dst,0,y,img_dst.cols,y,cv::Scalar(255,0,0));
                char str[20];
                sprintf(str,"%d",d_horizontal);
                putText(img_dst,str,cv::Point2i(0,y-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,0));
        }
        for(int d_vertical = -50; d_vertical<=50; d_vertical+=10) {
                int x = img_dst.cols/2+int(d_vertical*pixel_per_mm_x[res]);
                drawLine(img_dst,x,0,x,img_dst.rows,cv::Scalar(255,0,0));
                char str[20];
                sprintf(str,"%d",d_vertical);
                putText(img_dst,str,cv::Point2i(x+5,20),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,0));
        }
        drawLine(img_dst,0,mouse_cursor_camera.y(),
                 img_dst.cols,mouse_cursor_camera.y(),cv::Scalar(0,255,0),1);
        drawLine(img_dst,mouse_cursor_camera.x(),0,
                 mouse_cursor_camera.x(),img_dst.rows,cv::Scalar(0,255,0),1);
        if(qr_code_detected) {
                for (size_t j = 0; j < qr_code_corners.size(); j++) {
                        cv::Point p(qr_code_corners[j].get_u(),qr_code_corners[j].get_v());
                        cv::drawMarker(img_dst,p,cv::Scalar( 0, 255, 0 ),cv::MarkerTypes::MARKER_CROSS,30,2);
                        char str[20];
                        sprintf(str,"%ld",j);
                        putText(img_dst,str,p,cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,0));
                }
        }
        Q_EMIT HUDupdate(res);
}

void CNCGUI::FrameGrabberThread(){
        ROS_INFO("frame grabber thread START");
        while(ros::ok()) {
                if(grab_frames) {
                        if(!grabFrame(frame[LOW_RES], LOW_RES))
                                continue;
                        drawHUD(frame[LOW_RES], frame_reticle[LOW_RES]);
                        if(ui.scan_continuosly->isChecked())
                                ScanStitch(frame[LOW_RES],LOW_RES);
                }

        }
        ROS_INFO("frame grabber thread STOP");
}

void CNCGUI::drawImage(Mat &img, QLabel* label){
        Mat img_cpy;
        if(img.empty())
                return;
        cv::cvtColor(img,img_cpy,CV_BGR2RGB); //Qt reads in RGB whereas CV in BGR
        cv::resize(img_cpy, img_cpy, cv::Size(label->width(),label->height()), 0, 0, CV_INTER_LINEAR);
        QImage imdisplay((uchar*)img_cpy.data, img_cpy.cols, img_cpy.rows, img_cpy.step, QImage::Format_RGB888); //Converts the CV image into Qt standard format
        label->setPixmap(QPixmap::fromImage(imdisplay));//display the image in label that is created earlier
}

void CNCGUI::drawHUD(int res){
        drawImage(frame_reticle[res],ui.camera);
}

void CNCGUI::drawScanArea(){
        // drawImage(cnc_area_image[LOW_RES],ui.scan_area_lo_res);
        // drawImage(cnc_area_image[HIGH_RES],ui.scan_area_hi_res);
        cv::cvtColor(cnc_area_image[LOW_RES],cnc_area_image_reticle[LOW_RES],CV_BGR2RGB); //Qt reads in RGB whereas CV in BGR
        cv::resize(cnc_area_image_reticle[LOW_RES], cnc_area_image_reticle[LOW_RES],
                   cv::Size(ui.cnc_area->width(),ui.cnc_area->height()), 0, 0, CV_INTER_LINEAR);
        float scale_x = cnc_area_image[LOW_RES].cols/ui.cnc_area->width();
        float scale_y = cnc_area_image[LOW_RES].rows/ui.cnc_area->height();
        drawLine(cnc_area_image_reticle[LOW_RES],0,mouse_cursor_cnc_area.y(),
                 cnc_area_image_reticle[LOW_RES].cols,mouse_cursor_cnc_area.y(),cv::Scalar(0,255,0),1);
        drawLine(cnc_area_image_reticle[LOW_RES],mouse_cursor_cnc_area.x(),0,
                 mouse_cursor_cnc_area.x(),cnc_area_image_reticle[LOW_RES].rows,cv::Scalar(0,255,0),1);

        char str[20];
        sprintf(str,"%.3f %.3f",(mouse_cursor_cnc_area.x()*scale_x-lo_res.height/2)/pixel_per_mm_x[LOW_RES],
                cnc_y_dim-(mouse_cursor_cnc_area.y()*scale_y-lo_res.width/2)/pixel_per_mm_y[LOW_RES]);
        putText(cnc_area_image_reticle[LOW_RES],str,cv::Point2i(mouse_cursor_cnc_area.x()+5,mouse_cursor_cnc_area.y()-10),
                cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,0));
        cv::rectangle(cnc_area_image_reticle[LOW_RES],
                      cv::Point2i(mouse_cursor_cnc_area.x()-lo_res.height/scale_x/2,
                                  mouse_cursor_cnc_area.y()-lo_res.width/scale_y/2),
                      cv::Point2i(mouse_cursor_cnc_area.x()+lo_res.height/scale_x/2,
                                  mouse_cursor_cnc_area.y()+lo_res.width/scale_y/2),
                      cv::Scalar(0, 255, 0));
        QImage imdisplay2((uchar*)cnc_area_image_reticle[LOW_RES].data,
                          cnc_area_image_reticle[LOW_RES].cols, cnc_area_image_reticle[LOW_RES].rows,
                          cnc_area_image_reticle[LOW_RES].step, QImage::Format_RGB888); //Converts the CV image into Qt standard format
        ui.cnc_area->setPixmap(QPixmap::fromImage(imdisplay2));//display the image in label that is created earlier
}

void CNCGUI::scan(){
        if(sliceExists(slice)) {
                QMessageBox msgBox;
                msgBox.setText("The slice exists already");
                msgBox.setInformativeText("Do you want to erase it and scan again?");
                msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::Abort);
                msgBox.setDefaultButton(QMessageBox::Abort);
                int ret = msgBox.exec();
                if(ret==QMessageBox::Yes) {
                        clearSlice(slice);
                        cubes.clear();
                        cube_active.clear();
                        ninety_six_well_IDs.clear();
                        ninety_six_well_content.clear();
                }else{
                        ROS_INFO("abort scan");
                        return;
                }
        }
        scan_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::ScanThread, this));
        scan_thread->detach();
}

void CNCGUI::ScanThread(){
        ROS_INFO("scan thread START");
        ui.auto_focus->setChecked(false);
        // zero();
        geometry_msgs::Vector3 msg;
        bool dir = false;
        msg.x = 0;
        msg.y = cnc_y_dim;
        // lets scan the area using three frames in each direction
        int nr_images_x = (brain_sample_bottom_right.x-brain_sample_top_left.x)/(lo_res.height/pixel_per_mm_x[LOW_RES])+1;
        int nr_images_y = (brain_sample_top_left.y-brain_sample_bottom_right.y)/(lo_res.width/pixel_per_mm_y[LOW_RES])+1;
        float x_step = (brain_sample_bottom_right.x-brain_sample_top_left.x)/nr_images_x;
        float y_step = (brain_sample_top_left.y-brain_sample_bottom_right.y)/nr_images_y;
        for(float y=brain_sample_top_left.y; y>=brain_sample_bottom_right.y; y-=y_step) {
                msg.y = y;
                if(!dir) {
                        for(float x=brain_sample_top_left.x; x<=brain_sample_bottom_right.x+10; x+=x_step) {
                                msg.x = x;
                                if(!WaitForPositionReachedSave(msg,0.2,30)) {
                                        ROS_ERROR("could not reach scan position, aborting scan...");
                                        return;
                                }
                                Mat img, img_raw;
                                ros::Time t0 = ros::Time::now();
                                auto_focus();
                                ros::Duration d(0.5);
                                d.sleep();
                                while(!grabFrame(img,img_raw,LOW_RES) ) {
                                        ROS_WARN_THROTTLE(1,"waiting to get valid low res frame");
                                        if((ros::Time::now()-t0).toSec()>3) {
                                                ROS_ERROR("giving up");
                                                return;
                                        }
                                }

                                saveImage(LOW_RES,slice,values["pos_x"].back(),values["pos_y"].back(),img_raw,img);
                                auto_focus();
                                d.sleep();
                                while(!grabFrame(img,img_raw,HIGH_RES) ) {
                                        ROS_WARN_THROTTLE(1,"waiting to get valid high res frame");
                                        if((ros::Time::now()-t0).toSec()>3) {
                                                ROS_ERROR("giving up");
                                                return;
                                        }
                                }
                                saveImage(HIGH_RES,slice,values["pos_x"].back(),values["pos_y"].back(),img_raw,img);

                        }
                }else{
                        for(float x=brain_sample_bottom_right.x; x>=brain_sample_top_left.x-10; x-=x_step) {
                                msg.x = x;
                                if(!WaitForPositionReachedSave(msg,0.2,30)) {
                                        ROS_ERROR("could not reach scan position, aborting scan...");
                                        return;
                                }
                                Mat img, img_raw;
                                ros::Time t0 = ros::Time::now();
                                while(!grabFrame(img,img_raw,LOW_RES) ) {
                                        ROS_WARN_THROTTLE(1,"waiting to get valid low res frame");
                                        if((ros::Time::now()-t0).toSec()>3) {
                                                ROS_ERROR("giving up");
                                                return;
                                        }
                                }

                                saveImage(LOW_RES,slice,values["pos_x"].back(),values["pos_y"].back(),img_raw,img);
                                while(!grabFrame(img,img_raw,HIGH_RES) ) {
                                        ROS_WARN_THROTTLE(1,"waiting to get valid high res frame");
                                        if((ros::Time::now()-t0).toSec()>3) {
                                                ROS_ERROR("giving up");
                                                return;
                                        }
                                }
                                saveImage(HIGH_RES,slice,values["pos_x"].back(),values["pos_y"].back(),img_raw,img);
                        }
                }
                dir = !dir;
        }
        loadPlanImage();
        ROS_INFO("scan thread STOP");
}

void CNCGUI::Clean(){
        ROS_INFO("clean START");

        if(!WaitForPositionReachedSave(cleanser_pos,0.2,30)) {
                ROS_ERROR("could not reach cleaning position, aborting...");
                return;
        }
        ros::Time t0 = ros::Time::now();
        switch (cleanser_strategy) {
        case 0: {
                MoveToolSave(cleanser_pos.z-cleanser_tool_depth);
                std_msgs::Int32 msg;
                msg.data = cleanser_intensity/100.0f*6000;
                cleanser_pub.publish(msg);
                while((ros::Time::now()-t0).toSec()<cleanser_time) {
                        ROS_INFO_THROTTLE(1,"waiting for cleaning to finish");
                }
                msg.data = 0;
                cleanser_pub.publish(msg);
                break;
        }
        }

        MoveToolSave(-1);
        t0 = ros::Time::now();
        while((ros::Time::now()-t0).toSec()<cleanser_dwell_after_clean) {
                ROS_INFO_THROTTLE(1,"dwelling...");
        }
        ROS_INFO("clean STOP");
}

void CNCGUI::JoyStickContolThread(){
        ROS_INFO("joystick control thread START");
        while (read_event(js, &joystick_event) == 0 && ros::ok()) {
                switch (joystick_event.type)
                {
                case JS_EVENT_BUTTON:
                        // ROS_INFO("Button %u %s\n", joystick_event.number, joystick_event.value ? "pressed" : "released");
                        if(joystick_event.value) {
                                switch(joystick_event.number) {
                                case 8: { // HOME
                                        ROS_INFO("home button pressed");
                                        zero();
                                        break;

                                }
                                case 0: { // confirm
                                        ui.confirm->setChecked(true);
                                        break;
                                }
                                case 1: { // hold position, abort
                                        ROS_INFO("hold position");
                                        geometry_msgs::Vector3 msg;
                                        msg.x = values["pos_x"].back();
                                        msg.y = values["pos_y"].back();
                                        msg.z = values["pos_z"].back();
                                        motor_command.publish(msg);
                                        ui.abort->setChecked(true);
                                        break;
                                }
                                case 3: { // move z completely up
                                        ROS_INFO("move z up");
                                        geometry_msgs::Vector3 msg;
                                        msg.x = values["pos_x"].back();
                                        msg.y = values["pos_y"].back();
                                        msg.z = 0;
                                        motor_command.publish(msg);
                                        break;
                                }
                                default:
                                        /* Ignore other buttons. */
                                        break;
                                }
                        }else{
                                switch(joystick_event.number) {
                                case 0: { // confirm
                                        ui.confirm->setChecked(false);
                                        break;
                                }
                                case 1: { // hold position, abort
                                        ui.abort->setChecked(false);
                                        break;
                                }
                                default: {
                                        /* Ignore other buttons. */
                                        break;
                                }
                                }
                        }
                        break;
                case JS_EVENT_AXIS:
                        joystick_axis = get_axis_state(&joystick_event, joystick_axes);
                        if (joystick_axis < 3) {
                                // ROS_INFO("Axis %zu at (%6d, %6d)\n", joystick_axis,
                                //          joystick_axes[joystick_axis].x,
                                //          joystick_axes[joystick_axis].y);
                                geometry_msgs::Vector3 msg;
                                msg.x = values["pos_x"].back();
                                msg.y = values["pos_y"].back();
                                msg.z = values["pos_z"].back();
                                switch(joystick_axis) {
                                case 1: {
                                        float val = joystick_axes[joystick_axis].x/32768.0f*5;
                                        // ROS_INFO("x-axis %f",val);
                                        msg.x+=val;
                                        val = joystick_axes[joystick_axis].y/32768.0f*5;
                                        // ROS_INFO("y-axis %f",val);
                                        msg.y-=val;
                                        break;
                                }
                                case 0: {
                                        float val = joystick_axes[joystick_axis].y/32768.0f*5;
                                        ROS_INFO("z-axis %f",val);
                                        msg.z-=val;
                                        break;
                                }
                                }
                                motor_command.publish(msg);
                        }
                        break;
                default:
                        /* Ignore init events. */
                        break;
                }

        }
        ROS_INFO("joystick control thread STOP");
}

void CNCGUI::MoveTool(float z){
        ROS_INFO("move tool to %.1f", z);
        geometry_msgs::Vector3 msg;
        msg.x = values["pos_x"].back();
        msg.y = values["pos_y"].back();
        msg.z = z;
        motor_command.publish(msg);
}

bool CNCGUI::MoveToolSave(float z, float error, int timeout_sec){
        ROS_INFO("move tool to %.1f", z);
        geometry_msgs::Vector3 msg;
        msg.x = values["pos_x"].back();
        msg.y = values["pos_y"].back();
        msg.z = z;
        motor_command.publish(msg);
        ros::Time t0 = ros::Time::now();
        while(fabsf(values["pos_z"].back()-z)>error) {
                if((ros::Time::now()-t0).toSec()>timeout_sec) {
                        ROS_WARN("move tool timeout! moveing tool completely up!");
                        msg.x = values["pos_x"].back();
                        msg.y = values["pos_y"].back();
                        msg.z = 0;
                        motor_command.publish(msg);
                        return false;
                }
        }
        return true;
}

void CNCGUI::loadPlanImage(){
        float x_dim = (brain_sample_bottom_right.x-brain_sample_top_left.x);
        float y_dim = (brain_sample_top_left.y-brain_sample_bottom_right.y);
        if(x_dim<=0 || y_dim<=0) {
                ROS_ERROR("scan area does not make sense, please calibrate sample");
                return;
        }
        if(!sliceExists(slice)) {
                ROS_INFO("slice does not exist yet, please scan");
                return;
        }
        scan_area_image[HIGH_RES] = cv::Mat(cv::Size(int((x_dim*pixel_per_mm_x[HIGH_RES])+hi_res.height),
                                                     int((y_dim*pixel_per_mm_y[HIGH_RES])+hi_res.width)),CV_8UC3);
        scan_area_image[HIGH_RES].setTo(Scalar(255,255,255));
        ROS_INFO("scan size in mm %.1f %.1f -> in pixel %d %d", x_dim, y_dim, scan_area_image[HIGH_RES].cols,scan_area_image[HIGH_RES].rows);
        // scan_area_image[HIGH_RES] = cv::Mat(cv::Size(int((x_dim*pixel_per_mm_x[HIGH_RES])+lo_res.height),
        //                                              int((y_dim*pixel_per_mm_y[HIGH_RES])+lo_res.width)),CV_8UC3);
        // scan_area_image[HIGH_RES].setTo(Scalar(255,255,255));
        vector<float> x,y;
        vector<unsigned long> timestamps;
        vector<Mat> img;
        if(!getSlice(HIGH_RES,slice,x,y,timestamps,img)) {
                return;
        }

        for(int i=0; i<timestamps.size(); i++) {
                Stitch(HIGH_RES,x[i]-brain_sample_top_left.x,brain_sample_top_left.y-y[i],img[i]);
        }
        plan_image[HIGH_RES] = scan_area_image[HIGH_RES].clone();
        resize(plan_image[HIGH_RES], plan_image[LOW_RES],
               cv::Size(ui.plan_area->width(),ui.plan_area->height()), 0, 0, CV_INTER_LINEAR);
        plan_image_reticle[HIGH_RES] = plan_image[HIGH_RES].clone();
        plan_image_reticle[LOW_RES] = plan_image[LOW_RES].clone();
        drawPlan();
}

void CNCGUI::calculate96wellPositions(){
        ninety_six_well_pos.clear();
        for(int well=0; well<ninety_six_well_top_left.size(); well++) {
                for(int i=0; i<8; i++) {
                        for(int j=0; j<12; j++) {
                                ninety_six_well_pos.push_back(
                                        Point2f(ninety_six_well_top_left[well].x+i*ninety_six_well_distance,
                                                ninety_six_well_top_left[well].y-j*ninety_six_well_distance
                                                ));
                        }
                }
        }
        ROS_INFO("96well positions updated");
}

void CNCGUI::autoPlan(){
        Mat mask;
        int threshold = ui.auto_plan_threshold->value();
        ROS_INFO("threshold %d", threshold);
        inRange(plan_image[HIGH_RES], Scalar(0, 0, 0), Scalar(threshold, threshold, threshold), mask);
        cubes.clear();
        cube_active.clear();
        float x_dim = (brain_sample_bottom_right.x-brain_sample_top_left.x);
        float y_dim = (brain_sample_top_left.y-brain_sample_bottom_right.y);
        cubes_dim.width = int(x_dim/(tool_size+tool_offset)+1);
        cubes_dim.height = int(y_dim/(tool_size+tool_offset)+1);
        for(float cube_x = (hi_res.height/2/pixel_per_mm_x[HIGH_RES]); cube_x<x_dim+(hi_res.height/2/pixel_per_mm_x[HIGH_RES]); cube_x+=(tool_size+tool_offset)) {
                for(float cube_y = (hi_res.width/2/pixel_per_mm_y[HIGH_RES]); cube_y<y_dim+(hi_res.width/2/pixel_per_mm_y[HIGH_RES]); cube_y+=(tool_size+tool_offset)) {
                        cubes.push_back(Point2f(cube_x,cube_y));
                        cv::Mat pRoi = mask( cv::Rect( int(cube_x*pixel_per_mm_x[HIGH_RES]), int(cube_y*pixel_per_mm_y[HIGH_RES]),
                                                       int(tool_size*pixel_per_mm_x[HIGH_RES]), int(tool_size*pixel_per_mm_y[HIGH_RES])) );
                        if(cv::sum(pRoi)[0]!=0) {
                                cube_active.push_back(true);
                        }else{
                                cube_active.push_back(false);
                        }
                }
        }
        ROS_INFO_ONCE("%ld number of cubes, width %d height %d",cubes.size(),cubes_dim.width, cubes_dim.height);
        if(cubes_dim.width*cubes_dim.height!=cubes.size())
                ROS_ERROR("whoopsie, dimensions dont match");
        drawPlan();
}

bool CNCGUI::get96well(int cube, Scalar &color){
        int i=0;
        for(auto v:ninety_six_well_content) {
                for(auto val:v) {
                        if(val==cube) {
                                unsigned long id = ninety_six_well_IDs[i];
                                color = Scalar((id>>16)&0xff,(id>>8)&0xff,id&0xff);
                                return true;
                        }
                }
                i++;
        }
        return false;
}

void CNCGUI::drawPlan(){
        plan_image_reticle[LOW_RES] = plan_image[LOW_RES].clone();
        float conv_x = (float)ui.plan_area->width()/(float)plan_image[HIGH_RES].cols*pixel_per_mm_x[HIGH_RES];
        float conv_y = (float)ui.plan_area->height()/(float)plan_image[HIGH_RES].rows*pixel_per_mm_y[HIGH_RES];
        // ROS_INFO("%d cubes", cubes.size());
        for(int i=0; i<cubes.size(); i++) {
                Rect rect(int(cubes[i].x*conv_x),
                          int(cubes[i].y*conv_y),
                          int(tool_size*conv_x),
                          int(tool_size*conv_y));
                if(i==cube_hovered) {
                        if(plan_operation==0)
                                rectangle(plan_image_reticle[LOW_RES], rect, Scalar(255,0,0),5);
                        else if(plan_operation==1)
                                rectangle(plan_image_reticle[LOW_RES], rect, Scalar(0,255,0),5);
                        else if(plan_operation==2)
                                rectangle(plan_image_reticle[LOW_RES], rect, Scalar(0,0,255),5);
                }else if(i==cube_target) {
                        circle(plan_image_reticle[LOW_RES],
                               Point2i(int((cubes[i].x+tool_size/2)*conv_x),
                                       int((cubes[i].y+tool_size/2)*conv_y)),
                               10,Scalar(0,0,255),FILLED);
                }else{
                        if(cube_active[i]) {
                                Scalar color;
                                if(get96well(i,color)) {
                                        rectangle(plan_image_reticle[LOW_RES], rect, color,1);
                                        circle(plan_image_reticle[LOW_RES],
                                               Point2i(int((cubes[i].x+tool_size/2)*conv_x),
                                                       int((cubes[i].y+tool_size/2)*conv_y)),
                                               2,color,FILLED);
                                }else{
                                        rectangle(plan_image_reticle[LOW_RES], rect, Scalar(0,255,0),1);
                                        circle(plan_image_reticle[LOW_RES],
                                               Point2i(int((cubes[i].x+tool_size/2)*conv_x),
                                                       int((cubes[i].y+tool_size/2)*conv_y)),
                                               2,Scalar( 0, 0, 255 ),FILLED);
                                }
                        }else{
                                rectangle(plan_image_reticle[LOW_RES], rect, Scalar(20,20,20),1);
                        }
                }
        }
        drawImage(plan_image_reticle[LOW_RES],ui.plan_area);
}

void CNCGUI::Stitch(int res, float x, float y, Mat &img){
        int x_top_left = int(x*pixel_per_mm_x[res]),
            y_top_left = int(y*pixel_per_mm_y[res]);
        int border_x = 0, border_y = 0;
        int image_width = (res==LOW_RES ? lo_res.height : hi_res.height);
        int image_height = (res==LOW_RES ? lo_res.width : hi_res.width);
        if((x_top_left+image_width)>scan_area_image[res].cols)
                border_x = scan_area_image[res].cols-(x_top_left+image_width);
        if((y_top_left+image_height)>scan_area_image[res].rows)
                border_y = scan_area_image[res].rows-(y_top_left+image_height);
        int offset_x = 0, offset_y = 0;
        if(x_top_left>=0 && y_top_left>=0) {
                cv::Mat pRoi = scan_area_image[res]( cv::Rect( x_top_left+offset_x, y_top_left+offset_y,
                                                               image_width-border_x-offset_x, image_height-border_y-offset_y) );
                cv::Mat pRoi_new = img( cv::Rect( offset_x,offset_y, image_width-border_x-offset_x, image_height-border_y-offset_y) );
                // ROS_INFO_STREAM_THROTTLE(1, pRoi.size() << pRoi_new.size());
                pRoi = pRoi*0.01 + 0.99*pRoi_new;
        }else{
                ROS_WARN("did not stitch %.1f %.1f", x,y);
        }
}

void CNCGUI::ScanStitch( int res ){
        if(!grabFrame(frame[res], res))
                return;
        float x = values["pos_x"].back(), y = values["pos_y"].back();

        int x_top_left = int(x*pixel_per_mm_x[res]),
            y_top_left = int(fabsf(cnc_y_dim-y)*pixel_per_mm_y[res]);
        int border_x = 0, border_y = 0;
        int image_width = (res==LOW_RES ? lo_res.height : hi_res.height);
        int image_height = (res==LOW_RES ? lo_res.width : hi_res.width);
        if((x_top_left+image_width)>cnc_area_image[res].cols)
                border_x = cnc_area_image[res].cols-(x_top_left+image_width);
        if((y_top_left+image_height)>cnc_area_image[res].rows)
                border_y = cnc_area_image[res].rows-(y_top_left+image_height);
        int offset_x = 0, offset_y = 0;
        if(x_top_left>=0 && y_top_left>=0) {
                cv::Mat pRoi = cnc_area_image[res]( cv::Rect( x_top_left+offset_x, y_top_left+offset_y,
                                                              image_width-border_x-offset_x, image_height-border_y-offset_y) );
                cv::Mat pRoi_new = frame[res]( cv::Rect( offset_x,offset_y, image_width-border_x-offset_x, image_height-border_y-offset_y) );
                // ROS_INFO_STREAM_THROTTLE(1, pRoi.size() << pRoi_new.size());
                if(res==LOW_RES)
                        pRoi = pRoi*0.95 + 0.05*pRoi_new;
                else if(res==HIGH_RES)
                        pRoi = pRoi*0.01 + 0.99*pRoi_new;
                Q_EMIT scanAreaUpdate();
        }

}

void CNCGUI::ScanStitch( Mat &img, int res ){
        float x = values["pos_x"].back(), y = values["pos_y"].back();

        int x_top_left = int(x*pixel_per_mm_x[res]),
            y_top_left = int(fabsf(cnc_y_dim-y)*pixel_per_mm_y[res]);
        int border_x = 0, border_y = 0;
        int image_width = (res==LOW_RES ? lo_res.height : hi_res.height);
        int image_height = (res==LOW_RES ? lo_res.width : hi_res.width);
        if((x_top_left+image_width)>cnc_area_image[res].cols)
                border_x = cnc_area_image[res].cols-(x_top_left+image_width);
        if((y_top_left+image_height)>cnc_area_image[res].rows)
                border_y = cnc_area_image[res].rows-(y_top_left+image_height);
        int offset_x = 0, offset_y = 0;
        if(x_top_left>=0 && y_top_left>=0) {
                cv::Mat pRoi = cnc_area_image[res]( cv::Rect( x_top_left+offset_x, y_top_left+offset_y,
                                                              image_width-border_x-offset_x, image_height-border_y-offset_y) );
                cv::Mat pRoi_new = img( cv::Rect( offset_x,offset_y, image_width-border_x-offset_x, image_height-border_y-offset_y) );
                // ROS_INFO_STREAM_THROTTLE(1, pRoi.size() << pRoi_new.size());
                if(res==LOW_RES)
                        pRoi = pRoi*0.95 + 0.05*pRoi_new;
                else if(res==HIGH_RES)
                        pRoi = pRoi*0.01 + 0.99*pRoi_new;
                Q_EMIT scanAreaUpdate();
        }

}



void CNCGUI::plotData(){
        ui.position->graph(0)->setData(values["motor_state_sec"],values["pos_x"]);
        ui.position->graph(1)->setData(values["motor_state_sec"],values["pos_y"]);
        ui.position->graph(2)->setData(values["motor_state_sec"],values["pos_z"]);
        ui.velocity->graph(0)->setData(values["motor_state_sec"],values["vel_x"]);
        ui.velocity->graph(1)->setData(values["motor_state_sec"],values["vel_y"]);
        ui.velocity->graph(2)->setData(values["motor_state_sec"],values["vel_z"]);

        ui.position->rescaleAxes();
        ui.velocity->rescaleAxes();

        ui.position->replot();
        ui.velocity->replot();

        ui.x_pos->setText(QString::number(values["pos_x"].back()));
        ui.y_pos->setText(QString::number(values["pos_y"].back()));
        ui.z_pos->setText(QString::number(values["pos_z"].back()));
}

void CNCGUI::StatusReceiver(const sensor_msgs::JointStateConstPtr &msg){
        values["motor_state_sec"].push_back((msg->header.stamp-start_time).toSec());
        values["pos_x"].push_back(msg->position[0]);
        values["pos_y"].push_back(msg->position[1]);
        values["pos_z"].push_back(msg->position[2]);

        values["vel_x"].push_back(msg->velocity[0]);
        values["vel_y"].push_back(msg->velocity[1]);
        values["vel_z"].push_back(msg->velocity[2]);

        if(messages_received["motor_state"]>1000) {
                values["motor_state_sec"].pop_front();
                values["pos_x"].pop_front();
                values["pos_y"].pop_front();
                values["pos_z"].pop_front();
                values["vel_x"].pop_front();
                values["vel_y"].pop_front();
                values["vel_z"].pop_front();
        }else{
                messages_received["motor_state"]++;
        }

        static ros::Time t0 = ros::Time::now();

        if((ros::Time::now()-t0).toSec()>0.05) {
                t0 = ros::Time::now();
                Q_EMIT updateMotorState();
        }

}

void CNCGUI::move(){
        geometry_msgs::Vector3 msg;
        msg.x = values["pos_x"].back();
        msg.y = values["pos_y"].back();
        msg.z = values["pos_z"].back();
        if(ui.x_plus->isDown()) {
                msg.x += ui.step->text().toFloat();
        }else if(ui.x_minus->isDown()) {
                msg.x -= ui.step->text().toFloat();
        }else if(ui.y_plus->isDown()) {
                msg.y += ui.step->text().toFloat();
        }else if(ui.y_minus->isDown()) {
                msg.y -= ui.step->text().toFloat();
        }else if(ui.z_plus->isDown()) {
                msg.z += ui.step->text().toFloat();
        }else if(ui.z_minus->isDown()) {
                msg.z -= ui.step->text().toFloat();
        }
        motor_command.publish(msg);
}

void CNCGUI::dicingConfigUpdate(){
        ui.x_pos_96well_top_left_0->setText(QString::number(ninety_six_well_top_left[0].x));
        ui.y_pos_96well_top_left_0->setText(QString::number(ninety_six_well_top_left[0].y));
        ui.z_pos_96well_top_left_0->setText(QString::number(ninety_six_well_top_left[0].z));
        ui.x_pos_96well_top_left_1->setText(QString::number(ninety_six_well_top_left[1].x));
        ui.y_pos_96well_top_left_1->setText(QString::number(ninety_six_well_top_left[1].y));
        ui.z_pos_96well_top_left_1->setText(QString::number(ninety_six_well_top_left[1].z));
        ui.x_pos_96well_bottom_right_0->setText(QString::number(ninety_six_well_bottom_right[0].x));
        ui.y_pos_96well_bottom_right_0->setText(QString::number(ninety_six_well_bottom_right[0].y));
        ui.z_pos_96well_bottom_right_0->setText(QString::number(ninety_six_well_bottom_right[0].z));
        ui.x_pos_96well_bottom_right_1->setText(QString::number(ninety_six_well_bottom_right[1].x));
        ui.y_pos_96well_bottom_right_1->setText(QString::number(ninety_six_well_bottom_right[1].y));
        ui.z_pos_96well_bottom_right_1->setText(QString::number(ninety_six_well_bottom_right[1].z));
        ui.x_pos_brain_sample_top_left->setText(QString::number(brain_sample_top_left.x));
        ui.y_pos_brain_sample_top_left->setText(QString::number(brain_sample_top_left.y));
        ui.z_pos_brain_sample_top_left->setText(QString::number(brain_sample_top_left.z));
        ui.x_pos_brain_sample_bottom_right->setText(QString::number(brain_sample_bottom_right.x));
        ui.y_pos_brain_sample_bottom_right->setText(QString::number(brain_sample_bottom_right.y));
        ui.z_pos_brain_sample_bottom_right->setText(QString::number(brain_sample_bottom_right.z));
        ui.x_pos_cleanser->setText(QString::number(cleanser_pos.x));
        ui.y_pos_cleanser->setText(QString::number(cleanser_pos.y));
        ui.z_pos_cleanser->setText(QString::number(cleanser_pos.z));
        ui.x_pos_optical_reference->setText(QString::number(optical_reference_pos.x));
        ui.y_pos_optical_reference->setText(QString::number(optical_reference_pos.y));
        ui.z_pos_optical_reference->setText(QString::number(optical_reference_pos.z));
        ui.slice->setText(QString::number(slice));
        ui.cube->setText(QString::number(cube));
        ui.ninety_six_well_counter->setText(QString::number(ninety_six_well_counter));
        writeConfig(brainDiceConfigFile.toStdString());
}

void CNCGUI::planUpdate(){
        int active_positions = 0;
        ninety_six_well_IDs.clear();
        ninety_six_well_content.clear();
        vector<int> ninety_six_well;
        int i = 0;
        for(auto val:cube_active) {
                if(val) {
                        active_positions++;
                        ninety_six_well.push_back(i);
                        if(ninety_six_well.size()==96) { // if a well if full we take the next
                                ninety_six_well_IDs.push_back(ros::Time::now().toNSec());
                                ninety_six_well_content.push_back(ninety_six_well);
                                ninety_six_well.clear();
                        }
                }
                i++;
        }
        if(ninety_six_well.size()>0) { // if we started filling a 96 well and are done
                ninety_six_well_IDs.push_back(ros::Time::now().toNSec());
                ninety_six_well_content.push_back(ninety_six_well);
        }
        ROS_INFO("%d active positions distributed on %ld 96wells", active_positions, ninety_six_well_IDs.size());
        if(writePlan(slice,ros::Time::now().toNSec(),cubes_dim,cubes,cube_active,ninety_six_well_IDs,ninety_six_well_content)) {
                ROS_INFO("plan written");
        }
        drawPlan();
}

void CNCGUI::planUpdate(int write){
        if(write==1) {
                if(writePlan(slice,ros::Time::now().toNSec(),cubes_dim,cubes,cube_active,ninety_six_well_IDs,ninety_six_well_content))
                        ROS_INFO("plan written");
        }else{
                unsigned long timestamp;
                if(readPlan(slice,timestamp,cubes_dim,cubes,cube_active,ninety_six_well_IDs,ninety_six_well_content)) {
                        ROS_INFO("plan for slice %d with timestamp %ld successfully read", slice, timestamp);
                }
        }
        drawPlan();
}

void CNCGUI::zero(){
        std_srvs::Trigger msg;
        zero_srv.call(msg);
}

void CNCGUI::run(){
        if(run_thread) // maybe it's running already, wait until done...
                if(run_thread->joinable())
                        run_thread->join();
        run_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::RunThread, this));
        run_thread->detach();
}

void CNCGUI::pause(){
        if(ui.pause->isChecked()) {
                pause_active = false;
                ui.pause->setStyleSheet("background: lightgray");
        }else{
                pause_active = true;
                ui.pause->setStyleSheet("background: yellow");
        }
}

void CNCGUI::stop(){
        if(ui.stop->isChecked()) {
                stop_active = false;
                ui.stop->setStyleSheet("background: lightgray");
                if(ui.pause->isChecked()) {
                        ui.pause->setChecked(false);
                        ui.pause->setStyleSheet("background: lightgray");
                }
        }else{
                stop_active = true;
                ui.stop->setStyleSheet("background: red");
        }
}

void CNCGUI::prev_slice(){
        slice--;
        Q_EMIT updateDicingConfig();
        Q_EMIT updateButtonStates();
        Q_EMIT updatePlan(false);
}

void CNCGUI::next_slice(){
        slice++;
        Q_EMIT updateDicingConfig();
        Q_EMIT updateButtonStates();
        Q_EMIT updatePlan(false);
}

void CNCGUI::next_cube(){
        cube++;
        Q_EMIT updateDicingConfig();
        Q_EMIT updateButtonStates();
}

void CNCGUI::prev_cube(){
        cube--;
        Q_EMIT updateDicingConfig();
        Q_EMIT updateButtonStates();
}

void CNCGUI::scan_96well(){
        ROS_INFO("scan_96well button clicked");
}

void CNCGUI::swap_96well(){
        ROS_INFO("swap_96well button clicked");
}

void CNCGUI::buttonStateUpdate(){
        ui.prev_slice->setEnabled(slice>0);
        ui.prev_cube->setEnabled(cube>0);
}

void CNCGUI::confirm(){
        ROS_INFO("confirm");
}

void CNCGUI::dry_run(){
        if(dry_run_thread) // maybe it's running already, wait until done...
                if(dry_run_thread->joinable())
                        dry_run_thread->join();
        dry_run_thread = boost::shared_ptr<std::thread>( new std::thread(&CNCGUI::DryRunThread, this));
        dry_run_thread->detach();
}

void CNCGUI::BackLashCalibrationThread(){
        // ROS_INFO("starting backlash calibration thread");
        // camera_calibration = true;
        // ros::Duration d(1);
        // geometry_msgs::Vector3 msg;
        // vector<cv::Point2f> qr_code_positions;
        // cv::Point2f cnc_start_position(values["pos_x"].back(),values["pos_y"].back());
        // enum STATE {
        //         PLUS,
        //         CENTER,
        //         MINUS
        // };
        // int state = STATE::PLUS;
        // msg.y = cnc_start_position.y;
        // bool dir = false, mode = false;
        // float dx = 10;
        // vector<int> states;
        // for(int i=0; i<30; i++) {
        //         switch (state) {
        //         case STATE::PLUS:
        //                 if(!mode) {
        //                         msg.x = cnc_start_position.x+dx;
        //                         msg.y = cnc_start_position.y;
        //                 }else{
        //                         msg.x = cnc_start_position.x;
        //                         msg.y = cnc_start_position.y+dx;
        //                 }
        //                 break;
        //         case STATE::CENTER:
        //                 msg.x = cnc_start_position.x;
        //                 msg.y = cnc_start_position.y;
        //                 break;
        //         case STATE::MINUS:
        //                 if(!mode) {
        //                         msg.x = cnc_start_position.x-dx;
        //                         msg.y = cnc_start_position.y;
        //                 }else{
        //                         msg.x = cnc_start_position.x;
        //                         msg.y = cnc_start_position.y-dx;
        //                 }
        //                 break;
        //         }
        //         states.push_back(state);
        //         if(!dir) {
        //                 state++;
        //                 if(state>STATE::MINUS) {
        //                         state = STATE::CENTER;
        //                         dir = !dir;
        //                 }
        //         }else{
        //                 state--;
        //                 if(state<STATE::PLUS) {
        //                         state = STATE::CENTER;
        //                         dir = !dir;
        //                 }
        //         }
        //         WaitForPositionReached(msg,0.01,3);
        //         ros::Time t0 = ros::Time::now();
        //         float qr_x = 0, qr_y = 0;
        //         int samples = 0;
        //         while((ros::Time::now()-t0).toSec()<1) {
        //                 qr_x+=qr_code_pos[0];
        //                 qr_y+=qr_code_pos[1];
        //                 samples++;
        //         }
        //         qr_code_positions.push_back(cv::Point2f(qr_x/samples,qr_y/samples));
        //         // if(i>20)
        //         //   mode = true;
        // }
        // vector<float> dX = {0,0,0,0};
        // vector<int> samples = {0,0,0,0};
        // for(int i=1; i<qr_code_positions.size(); i++) {
        //         float dx = sqrtf(powf(qr_code_positions[i].x-qr_code_positions[i-1].x,2.0f)+powf(qr_code_positions[i].y-qr_code_positions[i-1].y,2.0f));
        //         if(states[i-1]==0 && states[i]==1 ) {
        //                 dX[0]+=dx;
        //                 samples[0]+=1;
        //         }else if(states[i-1]==1 && states[i]==2 ) {
        //                 dX[1]+=dx;
        //                 samples[1]+=1;
        //         }else if(states[i-1]==2 && states[i]==1 ) {
        //                 dX[2]+=dx;
        //                 samples[2]+=1;
        //         }else if(states[i-1]==1 && states[i]==0 ) {
        //                 dX[3]+=dx;
        //                 samples[3]+=1;
        //         }
        // }
        //
        // ROS_INFO("%.3f samples %d    %d->%d",dX[0]/samples[0],samples[0],0,1);
        // ROS_INFO("%.3f samples %d    %d->%d",dX[1]/samples[1],samples[1],1,2);
        // ROS_INFO("%.3f samples %d    %d->%d",dX[2]/samples[2],samples[2],2,1);
        // ROS_INFO("%.3f samples %d    %d->%d",dX[3]/samples[3],samples[3],1,0);
        //
        //
        // msg.x = cnc_start_position.x;
        // msg.y = cnc_start_position.y;
        // motor_command.publish(msg);
        //
        // ROS_INFO("stopping backlash calibration thread");
        //
        // // ros::Duration d(1);
        // // while(qr_tracking_error>0.5){
        // //   ROS_INFO_THROTTLE(1,"waiting for tracking error to be below 0.5, currently %f", qr_tracking_error);
        // //   d.sleep();
        // // }
        // // vector<geometry_msgs::Pose> qr_code_poses;
        // // vector<cv::Point2f> cnc_positions, qr_code_positions;
        // // cnc_positions.push_back(cv::Point2f(values["pos_x"].back(),values["pos_y"].back()));
        // // qr_code_positions.push_back(cv::Point2f(qr_code.position.x,qr_code.position.y));
        // // vector<float> x_pos = {-4,-8,4,8,0,0}, y_pos = {0,0,-4,-8,4,8};
        // // for(int i=0;i<x_pos.size();i++){
        // //   target.position.x = x_pos[i];
        // //   target.position.y = y_pos[i];
        // //   d.sleep();
        // //   while(qr_tracking_error>0.5){
        // //     ROS_INFO_THROTTLE(1,"waiting for tracking error to be below 0.5, currently %f", qr_tracking_error);
        // //     d.sleep();
        // //   }
        // //   cnc_positions.push_back(cv::Point2f(values["pos_x"].back()-cnc_positions[0].x-cnc_positions[i-1].x,values["pos_y"].back()-cnc_positions[0].y-cnc_positions[i-1].y));
        // //   qr_code_positions.push_back(cv::Point2f(qr_code.position.x-qr_code_positions[0].x-qr_code_positions[i-1].x,qr_code.position.y-qr_code_positions[0].y-qr_code_positions[i-1].y));
        // // }
        // // for(int i=0;i<cnc_positions.size();i++){
        // //   ROS_INFO("cnc(%.3f %.3f) qr(%.3f %.3f)",cnc_positions[i].x,cnc_positions[i].y,qr_code_positions[i].x,qr_code_positions[i].y);
        // // }
        //

}

PLUGINLIB_EXPORT_CLASS(CNCGUI, rqt_gui_cpp::Plugin)
