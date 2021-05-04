#include "TOF_Calibration_Types.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define PRINT_DEBUG

// Linked list of CAL_ADDRVAL_REG_BLOCK elements
struct CAL_ADDRVAL_REG_BLOCK_Node {
  struct CAL_ADDRVAL_REG_BLOCK_V1 *data;
  struct CAL_ADDRVAL_REG_BLOCK_Node *next;
};

// Linked list of CAL_GAIN_CORRECTION_BLOCK elements
struct CAL_GAIN_CORRECTION_BLOCK_Node {
  struct CAL_GAIN_CORRECTION_BLOCK *data;
  struct CAL_GAIN_CORRECTION_BLOCK_Node *next;
};

// Linked list of CAL_GEOMETRIC_BLOCK elements
struct CAL_GEOMETRIC_BLOCK_Node {
  struct CAL_GEOMETRIC_BLOCK_V3 *data;
  struct CAL_GEOMETRIC_BLOCK_Node *next;
};

// Linked list of CAL_RELATIVE_ILLUM_BLOCK elements
struct CAL_RELATIVE_ILLUM_BLOCK_Node {
  struct CAL_RELATIVE_ILLUM_BLOCK *data;
  struct CAL_RELATIVE_ILLUM_BLOCK_Node *next;
};

// Linked list of CAL_LSDAC_BLOCK elements
struct CAL_LSDAC_BLOCK_Node {
  struct CAL_LSDAC_BLOCK_V1 *data;
  struct CAL_LSDAC_BLOCK_Node *next;
};

// Linked list of CAL_P0BLOCK elements
struct CAL_P0BLOCK_Node {
  struct CAL_P0BLOCK_V4 *data;
  struct CAL_P0BLOCK_Node *next;
};

// Linked list of CAL_FOI_MASK_BLOCK elements
struct CAL_FOI_MASK_BLOCK_Node {
  struct CAL_FOI_MASK_BLOCK_V0_V1INFO *data;
  struct CAL_FOI_MASK_BLOCK_Node *next;
};

// Linked list of CAL_SPATIAL_TEMP_CORR_BLOCK elements
struct CAL_SPATIAL_TEMP_CORR_BLOCK_Node {
  struct CAL_SPATIAL_TEMP_CORR_BLOCK *data;
  struct CAL_SPATIAL_TEMP_CORR_BLOCK_Node *next;
};

// Linked list of CAL_ILLUM_PROFILE_BLOCK_V2 elements
struct CAL_ILLUM_PROFILE_BLOCK_Node {
  struct CAL_ILLUM_PROFILE_BLOCK_V2 *data;
  struct CAL_ILLUM_PROFILE_BLOCK_Node *next;
};

// Linked list of CAL_TEMP_CORR_BLOCK_V0_V1INFO elements
struct CAL_TEMP_CORR_BLOCK_Node {
  struct CAL_TEMP_CORR_BLOCK_V0_V1INFO *data;
  struct CAL_TEMP_CORR_BLOCK_Node *next;
};

// Structure containing linked lists for all calibration blocks
struct CAMERA_CAL {
  struct CAL_FILE_HEADER_V1 cal_file_header;
  struct CAL_HEADER_BLOCK_V3 cal_header_block;
  struct CAL_ADDRVAL_REG_BLOCK_Node *cal_addrval_reg_block_node;
  struct CAL_GAIN_CORRECTION_BLOCK_Node *cal_gain_correction_block_node;
  struct CAL_GEOMETRIC_BLOCK_Node *cal_geometric_block_node;
  struct CAL_RELATIVE_ILLUM_BLOCK_Node *cal_relative_illum_block_node;
  struct CAL_LSDAC_BLOCK_Node *cal_lsdac_block_node;
  struct CAL_P0BLOCK_Node *cal_p0block_node;
  struct CAL_FOI_MASK_BLOCK_Node *cal_foi_mask_block_node;
  struct CAL_SPATIAL_TEMP_CORR_BLOCK_Node *cal_spatial_temp_corr_block_node;
  struct CAL_ILLUM_PROFILE_BLOCK_Node *cal_illum_profile_block_node;
  struct CAL_TEMP_CORR_BLOCK_Node *cal_temp_corr_block_node;
};

int main(int argc, char *argv[]) {
  char filename[256] = "CCB_1909-0240034.ccb";

  // Open CCB file
  if(argc > 1)
    strcpy(filename, argv[1]);

  FILE *f = fopen(filename, "rb");
  if (f == NULL) {
    printf("Error opening CCB file %s\n", filename);
    exit(1);
  }

  // Declare and instation camera calibration structure
  struct CAMERA_CAL *camera_cal;
  camera_cal = (struct CAMERA_CAL *)calloc(1, sizeof(struct CAMERA_CAL));

  // Read calibration file header
  fread(&camera_cal->cal_file_header, sizeof(struct CAL_FILE_HEADER_V1), 1, f);

  // Read calibration header block
  fread(&camera_cal->cal_header_block, sizeof(struct CAL_HEADER_BLOCK_V3), 1,
        f);

  // Check for wrong header block
  if (camera_cal->cal_header_block.BlockInfo.BlockID != 0xFF) {
    printf("Wrong header block");
    exit(1);
  }

  // Declare linked lists and temporary calibration block pointers
  struct CAL_BLOCK_INFO_V1 cal_block_info;
  void *cal_block;
  struct CAL_ADDRVAL_REG_BLOCK_Node *cal_addrval_reg_block_node;
  struct CAL_GAIN_CORRECTION_BLOCK_Node *cal_gain_correction_block_node;
  struct CAL_GEOMETRIC_BLOCK_Node *cal_geometric_block_node;
  struct CAL_RELATIVE_ILLUM_BLOCK_Node *cal_relative_illum_block_node;
  struct CAL_LSDAC_BLOCK_Node *cal_lsdac_block_node;
  struct CAL_P0BLOCK_Node *cal_p0block_node;
  struct CAL_FOI_MASK_BLOCK_Node *cal_foi_mask_block_node;
  struct CAL_SPATIAL_TEMP_CORR_BLOCK_Node *cal_spatial_temp_corr_block_node;
  struct CAL_ILLUM_PROFILE_BLOCK_Node *cal_illum_profile_block_node;
  struct CAL_TEMP_CORR_BLOCK_Node *cal_temp_corr_block_node;

  // Loop over calibration blocks in CCB file
  for (uint16_t block_index = 0;
       block_index < camera_cal->cal_header_block.nBlocks; block_index++) {

    // Read calibration block information
    fread(&cal_block_info, sizeof(struct CAL_BLOCK_INFO_V1), 1, f);

    // Rewind file pointer to beginning of calibration blow information
    fseek(f, (int64_t)(-sizeof(struct CAL_BLOCK_INFO_V1)), SEEK_CUR);

    // Allocate memory for the calibration block
    cal_block = calloc(1,(size_t)cal_block_info.BlockSize);

    size_t bytes = fread(cal_block, 1, (size_t)(cal_block_info.BlockSize), f);
    if (cal_block_info.BlockSize != bytes)
      printf("error reading the cal_block %d from file\n", block_index);

    switch (cal_block_info.BlockID) {

    // Address/value Register List
    case 'A':
      printf("%d - Address/Value Register List \n", block_index);

      // Create new linked list node
      cal_addrval_reg_block_node = (struct CAL_ADDRVAL_REG_BLOCK_Node *)malloc(
          sizeof(struct CAL_ADDRVAL_REG_BLOCK_Node));
      cal_addrval_reg_block_node->data =
          (struct CAL_ADDRVAL_REG_BLOCK_V1 *)cal_block;
      cal_addrval_reg_block_node->next = camera_cal->cal_addrval_reg_block_node;

      // Push current list head down
      camera_cal->cal_addrval_reg_block_node = cal_addrval_reg_block_node;

      break;

    // FOI Mask
    case 'F':
      printf("%d - FOI Mask\n", block_index);

      // Create new linked list node
      cal_foi_mask_block_node = (struct CAL_FOI_MASK_BLOCK_Node *)malloc(
          sizeof(struct CAL_FOI_MASK_BLOCK_Node));
      cal_foi_mask_block_node->data =
          (struct CAL_FOI_MASK_BLOCK_V0_V1INFO *)cal_block;
      cal_foi_mask_block_node->next = camera_cal->cal_foi_mask_block_node;

      // Push current list head down
      camera_cal->cal_foi_mask_block_node = cal_foi_mask_block_node;

      break;

      // Geometric calibration
    case 'G':
      printf("%d - Geometric calibration\n", block_index);

      // Create new linked list node
      cal_geometric_block_node = (struct CAL_GEOMETRIC_BLOCK_Node *)malloc(
          sizeof(struct CAL_GEOMETRIC_BLOCK_Node));
      cal_geometric_block_node->data =
          (struct CAL_GEOMETRIC_BLOCK_V3 *)cal_block;
      cal_geometric_block_node->next = camera_cal->cal_geometric_block_node;

      // Push current list head down
      camera_cal->cal_geometric_block_node = cal_geometric_block_node;

      break;

    // Relative illumination
    case 'V':
      printf("%d - Relative Illumination\n", block_index);

      // Create new linked list node
      cal_relative_illum_block_node =
          (struct CAL_RELATIVE_ILLUM_BLOCK_Node *)malloc(
              sizeof(struct CAL_RELATIVE_ILLUM_BLOCK_Node));
      cal_relative_illum_block_node->data =
          (struct CAL_RELATIVE_ILLUM_BLOCK *)cal_block;
      cal_relative_illum_block_node->next =
          camera_cal->cal_relative_illum_block_node;

      // Push current list head down
      camera_cal->cal_relative_illum_block_node = cal_relative_illum_block_node;

      break;

    // Illumination Profile
    case 'I':
      printf("%d - Illumination Profile\n", block_index);

      // Create new linked list node
      cal_illum_profile_block_node =
          (struct CAL_ILLUM_PROFILE_BLOCK_Node *)malloc(
              sizeof(struct CAL_ILLUM_PROFILE_BLOCK_Node));
      cal_illum_profile_block_node->data =
          (struct CAL_ILLUM_PROFILE_BLOCK_V2 *)cal_block;
      cal_illum_profile_block_node->next =
          camera_cal->cal_illum_profile_block_node;

      // Push current list head down
      camera_cal->cal_illum_profile_block_node = cal_illum_profile_block_node;

      break;

    // LSDACs
    case 'L':

      printf("%d - LSDACs\n", block_index);

      // Create new linked list node
      cal_lsdac_block_node = (struct CAL_LSDAC_BLOCK_Node *)malloc(
          sizeof(struct CAL_LSDAC_BLOCK_Node));
      cal_lsdac_block_node->data = (struct CAL_LSDAC_BLOCK_V1 *)cal_block;
      cal_lsdac_block_node->next = camera_cal->cal_lsdac_block_node;

      // Push current list head down
      camera_cal->cal_lsdac_block_node = cal_lsdac_block_node;

      break;

    // P0
    case 'P':
      printf("%d - Complex Deprojection (P0)\n", block_index);

      // Create new linked list node
      cal_p0block_node =
          (struct CAL_P0BLOCK_Node *)malloc(sizeof(struct CAL_P0BLOCK_Node));
      cal_p0block_node->data = (struct CAL_P0BLOCK_V4 *)cal_block;
      cal_p0block_node->next = camera_cal->cal_p0block_node;

      // Push current list head down
      camera_cal->cal_p0block_node = cal_p0block_node;

      break;

    // Spatial Temperature Correction
    case 'S':
      printf("%d - Spatial Temperature Correction\n", block_index);

      // Create new linked list node
      cal_spatial_temp_corr_block_node =
          (struct CAL_SPATIAL_TEMP_CORR_BLOCK_Node *)malloc(
              sizeof(struct CAL_SPATIAL_TEMP_CORR_BLOCK_Node));
      cal_spatial_temp_corr_block_node->data =
          (struct CAL_SPATIAL_TEMP_CORR_BLOCK *)cal_block;
      cal_spatial_temp_corr_block_node->next =
          camera_cal->cal_spatial_temp_corr_block_node;

      // Push current list head down
      camera_cal->cal_spatial_temp_corr_block_node =
          cal_spatial_temp_corr_block_node;

      break;

    // Temperature Correction
    case 'T':
      printf("%d - Temperature Correction\n", block_index);

      // Create new linked list node
      cal_temp_corr_block_node = (struct CAL_TEMP_CORR_BLOCK_Node *)malloc(
          sizeof(struct CAL_TEMP_CORR_BLOCK_Node));
      cal_temp_corr_block_node->data =
          (struct CAL_TEMP_CORR_BLOCK_V0_V1INFO *)cal_block;
      cal_temp_corr_block_node->next = camera_cal->cal_temp_corr_block_node;

      // Push current list head down
      camera_cal->cal_temp_corr_block_node = cal_temp_corr_block_node;

      break;

    // Gain offset correction
    case 0xD7:
      printf("%d - Gain Correction\n", block_index);

      // Create new linked list node
      cal_gain_correction_block_node =
          (struct CAL_GAIN_CORRECTION_BLOCK_Node *)malloc(
              sizeof(struct CAL_GAIN_CORRECTION_BLOCK_Node *));
      cal_gain_correction_block_node->data =
          (struct CAL_GAIN_CORRECTION_BLOCK *)cal_block;
      cal_gain_correction_block_node->next =
          camera_cal->cal_gain_correction_block_node;

      // Push current list head down
      camera_cal->cal_gain_correction_block_node =
          cal_gain_correction_block_node;

      break;

    default:
      printf("Unknown block type %c \n", cal_block_info.BlockID);
      break;
    }
  }

#ifdef PRINT_DEBUG
  //  Print for debug (only partial information)

  // P0 Calibration
  cal_p0block_node = camera_cal->cal_p0block_node;
  while (cal_p0block_node != NULL) {
    printf("\n");
    printf("P0 Information\n");
    printf("--------------\n");
    printf("Nrows = %d\n", cal_p0block_node->data->nRows);
    printf("Ncols = %d\n", cal_p0block_node->data->nCols);

    for (size_t i = 0; i < cal_p0block_node->data->nRows; i++) {
      printf("%f ", cal_p0block_node->data->P0Coeffs[i]);
    }
    printf("\n");

    cal_p0block_node = cal_p0block_node->next;
  }

  // Gain correction
  cal_gain_correction_block_node = camera_cal->cal_gain_correction_block_node;
  while (cal_gain_correction_block_node != NULL) {
    printf("\n");
    printf("Gain calibration\n");
    printf("----------------\n");
    printf("Gain Cal Config Version = %d\n",
           cal_gain_correction_block_node->data->GainCalConfigVersion);
    printf("Subsampling Data Present = %d\n",
           cal_gain_correction_block_node->data->SubsamplingDataPresent);
    printf("Global ADC settings \n");
    printf("\t Iramp = %d\n", cal_gain_correction_block_node->data->ADC.IRamp);
    printf("\t Up/Down offset = %d\n",
           cal_gain_correction_block_node->data->ADC.UpdnoOffset);
    printf("Global ADC settings \n");
    printf("\t Vref1DAC = %d\n",
           cal_gain_correction_block_node->data->GainComparator.Vref1Dac);
    printf("A1D1\n");
    for (size_t i = 0; i < 10; i++) {
      printf(
          "%d ",
          cal_gain_correction_block_node->data->PerColOffsetAdjustment.A1D1[i]);
    }
    printf("\n");

    cal_gain_correction_block_node = cal_gain_correction_block_node->next;
  }

  // LSDACs
  cal_lsdac_block_node = camera_cal->cal_lsdac_block_node;
  while (cal_lsdac_block_node != NULL) {
    printf("\n");
    printf("LSDAC Calibration\n");
    printf("----------------\n");
    // Print subset of calibration block for debug
    printf("Nwrites = %d\n", cal_lsdac_block_node->data->nWrites);

    for (size_t i = 0; i < cal_lsdac_block_node->data->nWrites; i++) {
      printf("%d ", cal_lsdac_block_node->data->Settings[i]);
    }
    printf("\n");
    cal_lsdac_block_node = cal_lsdac_block_node->next;
  }

  // Relative illumination
  cal_relative_illum_block_node = camera_cal->cal_relative_illum_block_node;
  while (cal_relative_illum_block_node != NULL) {
    printf("\n");
    printf("Relative Illumination\n");
    printf("----------------\n");
    printf("Centroid X = %d\n", cal_relative_illum_block_node->data->CentroidX);
    printf("Nsamples = %d\n", cal_relative_illum_block_node->data->nSamples);

    for (size_t i = 0; i < cal_relative_illum_block_node->data->nSamples; i++) {
      printf("%d ", cal_relative_illum_block_node->data->Model[i]);
    }
    printf("\n");
    cal_relative_illum_block_node = cal_relative_illum_block_node->next;
  }

  // Geometric calibration
  cal_geometric_block_node = camera_cal->cal_geometric_block_node;
  while (cal_geometric_block_node != NULL) {
    printf("\n");
    printf("Geometric Calibration\n");
    printf("----------------\n");
    printf("Nrows = %d\n", cal_geometric_block_node->data->nRows);
    printf("Ncols = %d\n", cal_geometric_block_node->data->nCols);
    printf("Fc1 = %f\n", cal_geometric_block_node->data->Fc1);

    cal_geometric_block_node = cal_geometric_block_node->next;
  }

  // Address/Value Registers
  cal_addrval_reg_block_node = camera_cal->cal_addrval_reg_block_node;
  while (cal_addrval_reg_block_node != NULL) {
    printf("\n");
    printf("Address/Value Registers\n");
    printf("----------------\n");
    printf("Ntuples = %d\n", cal_addrval_reg_block_node->data->nTuples);
    for (size_t i = 0; i < cal_addrval_reg_block_node->data->nTuples; i++) {
      printf("Address = %d\t",
             cal_addrval_reg_block_node->data->Tuples[i].Address);
      printf("Value = %d\t", cal_addrval_reg_block_node->data->Tuples[i].Value);
      printf("Mask = %d\t", cal_addrval_reg_block_node->data->Tuples[i].Mask);
      printf("Comment = %d\n",
             cal_addrval_reg_block_node->data->Tuples[i].Comment);
    }
    cal_addrval_reg_block_node = cal_addrval_reg_block_node->next;
  }

  // FOI Mask
  cal_foi_mask_block_node = camera_cal->cal_foi_mask_block_node;
  while (cal_foi_mask_block_node != NULL) {
    printf("\n");
    printf("FOI Mask\n");
    printf("----------------\n");
    printf("nOffsetRows = %d\n", cal_foi_mask_block_node->data->nOffsetRows);
    printf("nOffsetCols = %d\n", cal_foi_mask_block_node->data->nOffsetCols);

    for (size_t i = 0; i < cal_foi_mask_block_node->data->nRows; i++) {
      printf("Begin Row = %d\n",
             cal_foi_mask_block_node->data->RowExtents[i].rowBegin);
      printf("End Row = %d\n",
             cal_foi_mask_block_node->data->RowExtents[i].rowEnd);
    }
    printf("\n");

    cal_foi_mask_block_node = cal_foi_mask_block_node->next;
  }

  // Spatial temperature correction
  cal_spatial_temp_corr_block_node =
      camera_cal->cal_spatial_temp_corr_block_node;
  while (cal_spatial_temp_corr_block_node != NULL) {
    printf("\n");
    printf("Spatial Temperature Correction\n");
    printf("----------------\n");
    printf("Freq = %d\n", cal_spatial_temp_corr_block_node->data->Freq);
    printf("Ingration time = %d\n",
           cal_spatial_temp_corr_block_node->data->IntegrationTime);

    for (size_t i = 0; i < 4; i++) {
      printf("Row temperature Coefficient = %f\n",
             cal_spatial_temp_corr_block_node->data->RowTemperatureCoeff[i]);
    }
    printf("\n");

    cal_spatial_temp_corr_block_node = cal_spatial_temp_corr_block_node->next;
  }

  // Illumination profile
  cal_illum_profile_block_node = camera_cal->cal_illum_profile_block_node;
  while (cal_illum_profile_block_node != NULL) {
    printf("\n");
    printf("Illumination profile\n");
    printf("----------------\n");
    printf("Sensor to LSOffset X = %d\n",
           cal_illum_profile_block_node->data->SensorToLSOffsetX);
    printf("Sensor to LSOffset Y = %d\n",
           cal_illum_profile_block_node->data->SensorToLSOffsetY);
    printf("Sensor to LSOffset Z = %d\n",
           cal_illum_profile_block_node->data->SensorToLSOffsetZ);
    printf("\n");

    cal_illum_profile_block_node = cal_illum_profile_block_node->next;
  }

  // Temperature correction
  cal_temp_corr_block_node = camera_cal->cal_temp_corr_block_node;
  while (cal_temp_corr_block_node != NULL) {
    printf("\n");
    printf("Temperature correction\n");
    printf("----------------\n");
    printf("Laser ID = %d\n", cal_temp_corr_block_node->data->LaserID);

    printf("Amplitude Coeff = \t");
    for (size_t i = 0; i < 8; i++) {
      printf("%f\t", cal_temp_corr_block_node->data->coeffs.AmplitudeCoeff[i]);
    }
    for (size_t i = 0; i < 8; i++) {
      printf("%f\t", cal_temp_corr_block_node->data->coeffs.PhaseCoeff[i]);
    }
    printf("\n");

    printf("\n");

    cal_temp_corr_block_node = cal_temp_corr_block_node->next;
  }
#endif

  // Free memory
  free(camera_cal);
  free(cal_block);
  free(cal_addrval_reg_block_node);
  free(cal_gain_correction_block_node);
  free(cal_geometric_block_node);
  free(cal_relative_illum_block_node);
  free(cal_lsdac_block_node);
  free(cal_p0block_node);
  free(cal_foi_mask_block_node);
  free(cal_spatial_temp_corr_block_node);
  free(cal_illum_profile_block_node);
  free(cal_temp_corr_block_node);
  return 0;
}
