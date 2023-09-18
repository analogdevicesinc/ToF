#include <stdio.h>
#include <command_parser.h>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <stdint.h>
#include <string.h>

#include "point_cloud.h"


#define MAX_FILE_NAME_SIZE 512


static char const* p_xyz_frame_output_file_name = "XYZFrame.bin";


static const char kUsage[] =
R"(PointCloud.
    kUsage:
      PointCloud --input_file=<Input depth file> --ccb_file=<ccb_file> --mode=<mode> --output_folder=<Output_folder>
      PointCloud (-h | --help) 
)";



int main(int argc, char* argv[]) {
        std::map<std::string, struct Argument> command_map = {
        {"-h", {"--help", false, "", ""}},
        {"-input_file", {"--input_file", true, "1", ""}},
        {"-ccb_file", {"--ccb_file", true, "2", ""}},
        {"-mode", {"--mode", true, "3", ""}},
        {"-output_folder", {"--output_folder", false, "", "./"}}};

    int status = 0;
    FILE* finput_depth_file = NULL;
    FILE* fXYZ = NULL;
    FILE* ccb_file = NULL;
    uint16_t* input_buffer = NULL;
    int16_t* p_xyz_frame = NULL;
    uint8_t* ccb_file_buff = NULL;

    char  input_file_path[MAX_FILE_NAME_SIZE], p_xyz_frame_output_file_path[MAX_FILE_NAME_SIZE];
    char  ccb_filename[MAX_FILE_NAME_SIZE];

    CommandParser command;
    std::string arg_error;
    command.parseArguments(argc, argv, command_map);

    int result = command.checkArgumentExist(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Argument " << arg_error << " doesn't exist! "
                   << "Please check help menu.";
        LOG(INFO) << kUsage;
        return -1;
    }

    result = command.helpMenu();
    if (result == 1) {
        LOG(INFO) << kUsage;
        return 0;
    } else if (result == -1) {
        LOG(ERROR) << "Usage of argument -h/--help"
                   << " is incorrect! Help argument should be used alone!";
        return -1;
    }

    result = command.checkValue(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Argument: " << command_map[arg_error].long_option
                   << " doesn't have assigned or default value!";
        LOG(INFO) << kUsage;
        return -1;
    }

    result = command.checkMandatoryArguments(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Mandatory argument: "
                   << command_map[arg_error].long_option << " missing";
        LOG(INFO) << kUsage;
        return -1;
    }

    result = command.checkMandatoryPosition(command_map, arg_error);
    if (result != 0) {
        LOG(ERROR) << "Mandatory argument "
                   << command_map[arg_error].long_option
                   << " is not on its correct position ("
                   << command_map[arg_error].position << ").";
        LOG(INFO) << kUsage;
        return -1;
    }

    int mode = std::stoi(command_map["-mode"].value);

    /*Input Depth file */
    snprintf(input_file_path, sizeof(input_file_path), "%s",
             command_map["-input_file"].value.c_str());


    /* ccb calibration_file*/
    snprintf(ccb_filename, sizeof(ccb_filename), "%s", command_map["-ccb_file"].value.c_str());

    
   std::string output_path = command_map["-output_folder"].value;

    snprintf(p_xyz_frame_output_file_path, sizeof(p_xyz_frame_output_file_path),
        "%s/%s", output_path.c_str(), p_xyz_frame_output_file_name);


    finput_depth_file = fopen(input_file_path, "rb");
    fXYZ = fopen(p_xyz_frame_output_file_path, "wb");
    ccb_file = fopen(ccb_filename, "rb");


    size_t size = 0;
    if (0 == fseek(ccb_file, 0L, SEEK_END)) {
        size = ftell(ccb_file);
        rewind(ccb_file);
    }
    ccb_file_buff = (uint8_t*)malloc(size);
    size_t fret = fread(ccb_file_buff, size, 1, ccb_file);
    if (fret != 1) {
        printf("\n Failed to read the ccb file");
        return 1;
    }
    FileData ccb_file_data = { (unsigned char*)ccb_file_buff, size };

    TofiCCBData ccb_data;
    GetCameraIntrinsics(&ccb_file_data, &ccb_data, mode);

    int n_cols = ccb_data.n_cols;
    int n_rows = ccb_data.n_rows;

    input_buffer = (uint16_t*)malloc(sizeof(uint16_t) * n_cols * n_rows);
    fret = fread(input_buffer, sizeof(uint16_t) * n_cols * n_rows, 1, finput_depth_file);
    if (fret != 1) {
        printf("\n Failed to read the input file");
        return 1;
    }
    

    p_xyz_frame = (int16_t*)malloc(sizeof(int16_t) * n_cols * n_rows * 3);

    float* p_x_table = NULL;
    float* p_y_table = NULL;
    float* p_z_table = NULL;


    XYZData  p_xyz_data;
    memset(&p_xyz_data, 0, sizeof(XYZData));

    p_xyz_data.n_cols = n_cols;
    p_xyz_data.n_rows = n_rows;

    status = GenerateXYZTables(
        &p_x_table, &p_y_table, &p_z_table, &(ccb_data.camera_intrinsics),
        ccb_data.n_sensor_rows, ccb_data.n_sensor_cols,
        n_rows, n_cols,
        ccb_data.n_offset_rows, ccb_data.n_offset_cols,
        ccb_data.row_bin_factor, ccb_data.col_bin_factor,
        GEN_XYZ_ITERATIONS);

    if (status != 0) return status;

   
    if (p_x_table != NULL && p_y_table != NULL && p_z_table != NULL) {
        p_xyz_data.p_x_table = p_x_table;
        p_xyz_data.p_y_table = p_y_table;
        p_xyz_data.p_z_table = p_z_table;
    }
    else {
        status = -1;
    }

    // Compute Point cloud 
    ComputeXYZ(input_buffer, &p_xyz_data, p_xyz_frame);

    if (status != 0)
    {
        printf("\n Unable to compute XYZ !!");
    }

    fret = fwrite(p_xyz_frame, sizeof(int16_t) * n_cols * n_rows * 3,1, fXYZ);
    if (fret != 1) {
        printf("\n Failed to write XYZ output");
        return 1;
    }

  
    FreeXYZTables(p_x_table, p_y_table, p_z_table);

    if (finput_depth_file != NULL)
        fclose(finput_depth_file);
    
    if (fXYZ != NULL)
        fclose(fXYZ);

    if (input_buffer != NULL)
        free(input_buffer);

    if (p_xyz_frame != NULL)
        free(p_xyz_frame);

    if (ccb_file != NULL)
        fclose(ccb_file);

    if (ccb_file_buff != NULL)
        free(ccb_file_buff);

      
    return 0;
}
