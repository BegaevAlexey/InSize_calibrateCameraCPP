///////////////////////////////////////////////////////////////////////////////////
// main.cpp - reading a chessboard’s width and height, reading and collecting
// the requested number of views, and calibrating the camera
// The program is founded on example of "Learning OpenCV 3"
//

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>


// the function of information
void help(char **argv);

/*MAIN FUNCTION*/
int main(int argc, char *argv[]) {

    unsigned int n_boards = 0;  // will be set by input list
    float image_sf = 0.5f;      // image scaling factor
    float delay = 1.f;
    unsigned int board_w = 0L;
    unsigned int board_h = 0L;

    if (argc < 4 || argc > 6) {
        std::cout << "\nERROR: Wrong number of input parameters\n";
        help(argv);
        return -1;
    }

    // count of corners of chessboard
    board_w = (unsigned int)atoi(argv[1]);
    board_h = (unsigned int)atoi(argv[2]);

    // count good images of chessboard
    n_boards = (unsigned int)atoi(argv[3]);

    if (argc > 4) {
        delay = atof(argv[4]);
    }
    if (argc > 5) {
        image_sf = atof(argv[5]);
    }

    unsigned long board_n;
    board_n = board_w * board_h;
    cv::Size board_sz = cv::Size(board_w, board_h);

    // GET FILENAME
    std::stringstream ss;
    std::string str;
    std::stack<std::string> nameFales;
    std::string pathDirectory ("../../data_for_projects/chesboard_112_2560x1140");
    for (boost::filesystem::recursive_directory_iterator it(pathDirectory), end; it != end; ++it) {
        if (it->path().extension() == ".jpg") {
            // get name file with path file
            ss << *it;
            // create name file
            str = ss.str();
            str = str.substr(1, str.size()-2);
            // add stack
            nameFales.push(str);
            // clear string stream
            ss.str(std::string());
        }
    }

    // ALLOCATE STORAGE
    //
    std::vector<std::vector<cv::Point2f> > image_points;
    std::vector<std::vector<cv::Point3f> > object_points;

    // Capture corner views: loop until we've got n_boards successful
    // captures (all corners on the board are found).
    //
    double last_captured_timestamp = 0;
    cv::Size image_size;
    cv::Mat image0, image;
    unsigned long counter = 0;

    while (image_points.size() < (size_t)n_boards && !nameFales.empty()) {

        image0 = cv::imread(nameFales.top());
        nameFales.pop();

        image_size = image0.size();
        cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);

        // Find the board
        //
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(image, board_sz, corners, cv::CALIB_CB_FAST_CHECK);

        // Draw it
        //
        drawChessboardCorners(image, board_sz, corners, found);

        // If we got a good board, add it to our data
        //
        if (found) {
            image ^= cv::Scalar::all(255);
            cv::Mat mcorners(corners);

            // do not copy the data
            mcorners *= (1.0 / image_sf);

            // scale the corner coordinates
            image_points.push_back(corners);
            object_points.push_back(std::vector<cv::Point3f>());
            std::vector<cv::Point3f> &opts = object_points.back();

            opts.resize(board_n);
            for (int j = 0; j < board_n; j++) {
                opts[j] = cv::Point3f((float)(j / board_w),
                                      (float)(j % board_w), 0.0f);
            }
            std::cout << "Collected our " << (int)image_points.size()
                      << " of " << n_boards << " needed chessboard images\n" << std::endl;
        }
    }
    // END COLLECTION WHILE LOOP.

    std::cout << "\n\n*** CALIBRATING THE CAMERA...\n" << std::endl;

    // CALIBRATE THE CAMERA!
    //
    cv::Mat intrinsic_matrix, distortion_coeffs;
    double err = cv::calibrateCamera(
            object_points, image_points, image_size, intrinsic_matrix,
            distortion_coeffs, cv::noArray(), cv::noArray(),
            cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);

    // SAVE THE INTRINSICS AND DISTORTIONS
    std::cout << "*** DONE!\n\nReprojection error is " << err
         << "\nStoring Intrinsics.xml and Distortions.xml files\n\n";
    cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);
    fs << "image_width" << image_size.width << "image_height" << image_size.height
       << "camera_matrix" << intrinsic_matrix << "distortion_coefficients"
       << distortion_coeffs;
    fs.release();

    // EXAMPLE OF LOADING THESE MATRICES BACK IN:
    fs.open("intrinsics.xml", cv::FileStorage::READ);
    std::cout << "\nimage width: " << (int)fs["image_width"];
    std::cout << "\nimage height: " << (int)fs["image_height"];
    cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
    fs["camera_matrix"] >> intrinsic_matrix_loaded;
    fs["distortion_coefficients"] >> distortion_coeffs_loaded;
    std::cout << "\nintrinsic matrix:\n" << intrinsic_matrix_loaded;
    std::cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << std::endl;

    return 0;
}
/*END MAIN*/

// the function of information
void help(char **argv) {  // todo rewrite this
    std::cout << "\n"
              << "\nReading a chessboard’s width and height,\n"
              << "              reading and collecting the requested number of views,\n"
              << "              and calibrating the camera\n\n"
              << "Call:\n" << argv[0] << " <board_width> <board_height> <number_of_boards> <if_video,_delay_between_framee_capture> <image_scaling_factor>\n\n"
              << "Example:\n" << argv[0] << " 9 6 15 500 0.5\n"
              << "-- to use the checkerboard9x6.png provided\n\n"
              << " * First it reads in checker boards and calibrates itself\n"
              << " * Then it saves and reloads the calibration matricies\n"
              << " * Then it creates an undistortion map and finally\n"
              << " * It displays an undistorted image\n"
              << std::endl;
}

