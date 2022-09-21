#include <stdio.h>
#include "docopt.h"
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

    int status = 0;
    FILE* finput_depth_file = NULL;
    FILE* fXYZ = NULL;
    FILE* ccb_file = NULL;
    uint16_t* input_buffer = NULL;
    int16_t* p_xyz_frame = NULL;
    uint8_t* ccb_file_buff = NULL;

    char  input_file_path[MAX_FILE_NAME_SIZE], p_xyz_frame_output_file_path[MAX_FILE_NAME_SIZE];
    char  ccb_filename[MAX_FILE_NAME_SIZE];

    std::map<std::string, docopt::value> args = docopt::docopt(kUsage, { argv + 1, argv + argc }, true);

    

    int mode = (int)args["--mode"].asLong();

    /*Input Depth file */
    snprintf(input_file_path, sizeof(input_file_path), "%s", args["--input_file"].asString().c_str());


    /* ccb calibration_file*/
    snprintf(ccb_filename, sizeof(ccb_filename), "%s",args["--ccb_file"].asString().c_str());

    
   std::string output_path = args["--output_folder"].asString();

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
