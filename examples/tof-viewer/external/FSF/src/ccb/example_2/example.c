#include <ccb.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

ccb_data_t load_ccb_file(const char * const ccb_filename)
{
    void *p = 0;
    size_t size = 0;
    FILE *f = fopen(ccb_filename, "rb");
    if (f) {
        if (0 == fseek(f, 0L, SEEK_END)) {
            size = ftell(f);
            rewind(f);
            p = malloc(size);
            if (p) {
                if (1 != fread(p, size, 1, f)) {
                    fprintf(stderr, "Failed to read data file.\n");
                    free(p);
                    p = 0;
                    size = 0;
                }
            } else {
                fprintf(stderr, "Failed to allocate memory for CCB data.\n");
            }
        } else {
            fprintf(stderr, "Failed to seek in data file.\n");
        }
        fclose(f);
    } else {
        fprintf(stderr, "Failed to open data file %s.\n", ccb_filename);
    }
    return (ccb_data_t) {
        p, 
        size
    };
}

void write_CAL_FILE_HEADER_V1(FILE *f, const struct CAL_FILE_HEADER_V1 *header)
{
    fprintf(f, "CAL_FILE_HEADER_V1: {\n"
            "  CalibFileVersion: 0x%"PRIx16",\n"
            "  CalibVersion: 0x%"PRIx16",\n"
            "  CalibHdrSize: %"PRIu16",\n"
            "  rsvd: %"PRIu8",\n"
            "  CalibFileHdrSize: %"PRIu8",\n"
            "  CalibFileSize: %"PRIu32",\n"
            "  rsvd1: %"PRIu32"\n"
            "}\n",
            header->CalibFileVersion,
            header->CalibVersion,
            header->CalibHdrSize,
            header->rsvd,
            header->CalibFileHdrSize,
            header->CalibFileSize,
            header->rsvd1
           );
}

void write_CAL_HEADER_BLOCK_V3(FILE *f, const struct CAL_HEADER_BLOCK_V3 *header_block)
{
    fprintf(f, "CAL_HEADER_BLOCK_V3: {\n"
            // struct CAL_BLOCK_INFO_V1 BlockInfo;
            "  nBlocks: %"PRIu16",\n"
            "  ChipID: 0x%"PRIx16",\n"
            "  nRows: %"PRIu16",\n"
            "  nCols: %"PRIu16",\n"
            "  ConfigVersionStr: %.*s,\n"
            "  SerialNumber: %.*s,\n"
            "  ChipUniqueID: %.*s,\n"
            "  ControlAPIVersion: 0x%"PRIx32",\n"
            "  FirmwareVersion: 0x%"PRIx32",\n"
            "  CalibrationVersion: 0x%"PRIx32",\n"
            "  SequenceVersion: 0x%"PRIx32",\n"
            "}\n",
            header_block->nBlocks,
            header_block->ChipID,
            header_block->nRows,
            header_block->nCols,
            (int) sizeof(header_block->ConfigVersionStr), header_block->ConfigVersionStr,
            (int) sizeof(header_block->SerialNumber), header_block->SerialNumber,
            (int) sizeof(header_block->ChipUniqueID), header_block->ChipUniqueID,
            header_block->ControlAPIVersion,
            header_block->FirmwareVersion,
            header_block->CalibrationVersion,
            header_block->SequenceVersion
           );
}

int main(int argc, const char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Provide path to .ccb file.\n");
        return -1;
    }
    const char *ccb_filename = argv[1];
    ccb_data_t ccb_data = load_ccb_file(ccb_filename);
    if (!ccb_data.p) {
        fprintf(stderr, "Failed to load data file.\n");
        return -1;
    }
    const struct CAL_FILE_HEADER_V1 * header = ccb_read_header(&ccb_data);
    if (header) {
        write_CAL_FILE_HEADER_V1(stdout, header);
    }
    const struct CAL_HEADER_BLOCK_V3 *header_block = ccb_read_header_block(&ccb_data);
    if (header_block) {
        write_CAL_HEADER_BLOCK_V3(stdout, header_block);
    }
    int block_count = 0;
    const struct CAL_BLOCK_INFO_V1 *cal_block_info;
    for (int i = 0; cal_block_info = ccb_get_cal_block_info(&ccb_data, BLOCK_ALL, i); i++) {
        block_count += 1;
    }
    printf("Found %d cal blocks.\n", block_count);
    const struct CAL_ADDRVAL_REG_BLOCK_V1 *addrval_reglist;
    for (int i = 0; addrval_reglist = ccb_get_cal_block_addrval_reglist(&ccb_data, i); i++) {
        printf("addrval_reglist[%d]: %p\n", i, addrval_reglist);
    }
    const struct CAL_FOI_MASK_BLOCK_V0_V1INFO *foi_mask;
    for (int i = 0; foi_mask = ccb_get_cal_block_foi_mask(&ccb_data, i); i++) {
        printf("foi_mask[%d]: %p\n", i, foi_mask);
    }
    const struct CAL_GEOMETRIC_BLOCK_V3 *geometric;
    for (int i = 0; geometric = ccb_get_cal_block_geometric(&ccb_data, i); i++) {
        printf("geometric[%d]: %p\n", i, geometric);
    }
    const struct CAL_ILLUM_PROFILE_BLOCK_V2 *illum_profile;
    for (int i = 0; illum_profile = ccb_get_cal_block_illum_profile(&ccb_data, i); i++) {
        printf("illum_profile[%d]: %p\n", i, illum_profile);
    }
    const struct CAL_LSDAC_BLOCK_V1 *lsdacs;
    for (int i = 0; lsdacs = ccb_get_cal_block_lsdacs(&ccb_data, i); i++) {
        printf("lsdacs[%d]: %p\n", i, lsdacs);
    }
    const struct CAL_P0BLOCK_V4 *p0;
    for (int i = 0; p0 = ccb_get_cal_block_p0(&ccb_data, i); i++) {
        printf("p0[%d]: %p\n", i, p0);
    }
    const struct CAL_SPATIAL_TEMP_CORR_BLOCK *spatial_tempcor;
    for (int i = 0; spatial_tempcor = ccb_get_cal_block_spatial_tempcor(&ccb_data, i); i++) {
        printf("spatial_tempcor[%d]: %p\n", i, spatial_tempcor);
    }
    const struct CAL_TEMP_CORR_BLOCK_V0_V1INFO *tempcorrection;
    for (int i = 0; tempcorrection = ccb_get_cal_block_tempcorrection(&ccb_data, i); i++) {
        printf("tempcorrection[%d]: %p\n", i, tempcorrection);
    }
    const struct CAL_RELATIVE_ILLUM_BLOCK *relative_illum;
    for (int i = 0; relative_illum = ccb_get_cal_block_relative_illum(&ccb_data, i); i++) {
        printf("relative_illum[%d]: %p\n", i, relative_illum);
    }
    const struct CAL_GAIN_CORRECTION_BLOCK *gaincorrection;
    for (int i = 0; gaincorrection = ccb_get_cal_block_gaincorrection(&ccb_data, i); i++) {
        printf("gaincorrection[%d]: %p\n", i, gaincorrection);
    }
    const size_t BUFFER_SIZE = sizeof(ccb_index_t) + block_count * 8;
    void *buffer = malloc(BUFFER_SIZE);
    if (buffer) {
        const ccb_index_t *ccb_index = ccb_build_cal_block_index(&ccb_data, buffer, BUFFER_SIZE);
        if (ccb_index) {
            printf("num_blocks: %d\n", ccb_index->num_blocks);
            printf("num_addrval_reglists: %lu\n", ccb_index->num_addrval_reglists);
            printf("num_foi_masks: %lu\n", ccb_index->num_foi_masks);
            printf("num_geometrics: %lu\n", ccb_index->num_geometrics);
            printf("num_illum_profiles: %lu\n", ccb_index->num_illum_profiles);
            printf("num_lsdacss: %lu\n", ccb_index->num_lsdacss);
            printf("num_p0s: %lu\n", ccb_index->num_p0s);
            printf("num_spatial_tempcors: %lu\n", ccb_index->num_spatial_tempcors);
            printf("num_tempcorrections: %lu\n", ccb_index->num_tempcorrections);
            printf("num_relative_illums: %lu\n", ccb_index->num_relative_illums);
            printf("num_gaincorrections: %lu\n", ccb_index->num_gaincorrections);
            for (int i = 0; i < ccb_index->num_illum_profiles; i++) {
                const struct CAL_ILLUM_PROFILE_BLOCK_V2 *illum_profile = ccb_index->illum_profiles[i];
                printf("illum_profile[%d] (%p) nRows: %"PRIu16", nCols: %"PRIu16"\n",
                       i, illum_profile, illum_profile->nRows, illum_profile->nCols);
            }
        }
        free(buffer);
    }
    free((void *) ccb_data.p);
    printf("goodbye\n");
    return 0;
}
