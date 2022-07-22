# Overview #

## Brief Description ##

This repository is a stab at a C API for accessing binary CCB data.

# Installation #

## Dependencies ##

To build ccb, you need cmake and a C compiler.

## Portability ##

The ccb library should be portable to any C system. It assumes that the CCB binary data has the same endianness as the machine.

# Usage #

This repository provides a ccb source files and an example application that uses these source files. The example application must be pointed at a CCB file as a command line argument. Please see examples/CMakeLists.txt for an example of how to create an application with cmake that uses the ccb library.

To build and run the example:

```bash
ccb$ mkdir build
ccb$ cd build
ccb/build$ cmake ..
ccb/build$ cmake --build .
ccb/build$ cd Debug
ccb/build/Debug$ .\ccb_example.exe <path to CCB file>
```

## API Documentation ##

There is no API documentation at this point. However, here is the gist of it...

```C
#include <CCB/ccb.h>
```

Initialize a `ccb_data_t` that holds the CCB binary data. You could initialize the pointer member from memory or by reading the data from a file. The example application loads it from a file. The size member indicates how many bytes are in the binary data.

```C

typedef struct
{
    unsigned char *p;
    size_t size;
} ccb_data_t;
```

Then, you can use the functions in CCB/ccb.h to read the header data, and the block data either generically or with specific types. The functions that return block information take a zero-based index, and they return the nth block of the provided type. If there are fewer blocks than the index, then they return 0. The example application shows how to read all blocks of each type.

The `ccb_get_cal_block_info` function takes a block type, which should be one of the 'block_t' values from `ccb.h` such as `BLOCK_ADDRVAL_REG_BLOCK_V1`. You can also use `BLOCK_ALL` to index all blocks of all types in the order they are in the CCB data.

```C
const struct CAL_FILE_HEADER_V1 *ccb_read_header(const ccb_data_t * const ccb_data);
const struct CAL_HEADER_BLOCK_V3 *ccb_read_header_block(const ccb_data_t * const ccb_data);
const struct CAL_BLOCK_INFO_V1 *ccb_get_cal_block_info(const ccb_data_t * const ccb_data, const block_t block_id, const size_t index);
const struct CAL_ADDRVAL_REG_BLOCK_V1 *ccb_get_cal_block_addrval_reglist(const ccb_data_t * const ccb_data, const size_t index);
const struct CAL_FOI_MASK_BLOCK_V0_V1INFO *ccb_get_cal_block_foi_mask(const ccb_data_t * const ccb_data, const size_t index);
const struct CAL_GEOMETRIC_BLOCK_V3 *ccb_get_cal_block_geometric(const ccb_data_t * const ccb_data, const size_t index);
const struct CAL_ILLUM_PROFILE_BLOCK_V2 *ccb_get_cal_block_illum_profile(const ccb_data_t * const ccb_data, const size_t index);
const struct CAL_LSDAC_BLOCK_V1 *ccb_get_cal_block_lsdacs(const ccb_data_t * const ccb_data, const size_t index);
const struct CAL_P0BLOCK_V4 *ccb_get_cal_block_p0(const ccb_data_t * const ccb_data, const size_t index);
const struct CAL_SPATIAL_TEMP_CORR_BLOCK *ccb_get_cal_block_spatial_tempcor(const ccb_data_t * const ccb_data, const size_t index);
const struct CAL_TEMP_CORR_BLOCK_V0_V1INFO *ccb_get_cal_block_tempcorrection(const ccb_data_t * const ccb_data, const size_t index);
const struct CAL_RELATIVE_ILLUM_BLOCK *ccb_get_cal_block_relative_illum(const ccb_data_t * const ccb_data, const size_t index);
const struct CAL_GAIN_CORRECTION_BLOCK *ccb_get_cal_block_gaincorrection(const ccb_data_t * const ccb_data, const size_t index);
```

The libary does zero dynamic memory allocation, so the returned pointers should not be freed. The results have the same lifetime as the pointer member of the ccb_data_t structure. It is assumed that the binary data is constant, so it should not be modified.

The library also offers indexing of the CCB data. An index permits you to access the blocks of each type more efficiently than iterating to find them with the functions above.

To build an index, you provide a byte buffer to the `ccb_build_cal_block_index` function. The buffer must be at least as large as a `ccb_data_t`. It needs to have another pointer size bytes for each cal block that it stores. If the provided buffer is not at least as large as `ccb_data_t` then the function returns 0. Otherwise, it returns a pointer to a `ccb_index_t`. The `num_blocks` member indicates how many blocks were found in the CCB file. If it is negative number, then the buffer was not large enough to index all of the CCB blocks, and the absolute value indicates how many blocks the CCB data contains. You could use this to allocate a buffer large enough to hold a complete index. The function shall complete as much of the index as fits in the buffer.

The index has two members for each cal block type. The first is a pointer to the first of a consecutive list of pointers to blocks of the relevant type. The second is a quantity of pointers in that list. You can access the blocks as demonstrated in this snippet:

```C
for (int i = 0; i < ccb_index->num_illum_profiles; i++) {
    const struct CAL_ILLUM_PROFILE_BLOCK_V2 *illum_profile = ccb_index->illum_profiles[i];
    printf("illum_profile[%d] nRows: %"PRIu16", nCols: %"PRIu16"\n",
           i, illum_profile->nRows, illum_profile->nCols);
}
```

# Licensing #

This code is created by ADI.

## Maintenance Intentions ##

None at the moment. This is an experiment.

# Contact List #

Jason King

Nicolas Le Dortz
