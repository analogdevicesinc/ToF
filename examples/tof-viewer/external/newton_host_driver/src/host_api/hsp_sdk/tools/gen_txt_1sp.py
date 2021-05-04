import sys
import os
import struct
import argparse
from collections import OrderedDict

def get_cmd_1sp_header(len):
    hdr = '\n'.join([
        '0000', # Address
        '100A', # Command: 1SP
        '0001', # Attribute: Write
        f'{len:04X}', # Payload Size
    ])
    return hdr

def read_fuse_file(fuse_entry, filename):
    hex = open(filename, 'r').read().strip()
    bin = bytes.fromhex(hex)
    #print(fuse_entry, filename, bin)
    return bin

def override_prod1sp_fuse_data(body, fuse_files):
    print("Overriding fuse data in Production 1SP image with", ','.join(fuse_files))
    assert len(fuse_files) == 5, "Provide 5 fuse files: svn, public hash, encryption key, global private key, disable modes"
    fuse_file_byte_sizes = [
        ["SVN", 8],
        ["PublicHash", 32],
        ["EncryptionKey", 32],
        ["GlobalPrivateKey", 32],
        ["BlockedOperationModes", 4]
    ]
    fuse_block = b''
    for (fuse_entry, size), fuse_file in zip(fuse_file_byte_sizes, fuse_files):
        fuse_data = read_fuse_file(fuse_entry, fuse_file)
        assert len(fuse_data) == size, f"{fuse_entry} file {fuse_file} should be hex file with lenth {size}"
        fuse_block += fuse_data
    
    # Write SVN to Prod 1SP header - Production 1SP should have matching SVN that it writes
    # svn = read_fuse_file(fuse_entry, fuse_files[0])
    fuse_offset = 0     # Offset relative to prod_1sp.lds _1sp_image_start 
    # header = svn + header[len(svn):]
    body = body[:fuse_offset] + fuse_block + body[fuse_offset+len(fuse_block):]
    return body

def write_bin_in_newton_format(out_file, data):
    for i in range(0, len(data), 2):
        line = f'\n{data[i+1]:02X}{data[i]:02X}' # Swap every two bytes
        out_file.write(line)

#
# input_bin_fn already contains 1SP header + 1SP Code + 1SP Text
#
def bin2newton_cmd(input_bin_fn, output_txt_fn, prod1sp_fuse_files=[]):
    input_data = open(input_bin_fn, 'rb').read()
    input_size = len(input_data)
    print(input_size, input_data[:10])
    assert (input_size % 4 == 0), "Input binary should be 4 bytes aligned"


    # if prod1sp_fuse_files: 
        # if input_size < 0x4100:
            # print('Padding 1SP image')
            # input_data = input_data + b'\x00' * (0x4100 - input_size)
            # assert len(input_data) == 0x4100
            # input_size = len(input_data)
        # Will need to pass body once header relocation of 1SP image to top of RAM
        # input_data = override_prod1sp_fuse_data(input_data, prod1sp_fuse_files)

    header = input_data[:512]
    body = input_data[512:]

    # TODO: Override header section based on SP_Bin_Format_Header_t structure
    # Not used for now
    # uint64_t                    SVN; 
    # uint32_t                    Image1SPBodySize;

    with open(output_txt_fn, 'w') as out_file:
        out_file.write(get_cmd_1sp_header(input_size))
        write_bin_in_newton_format(out_file, header)
        write_bin_in_newton_format(out_file, body)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("src_bin", type=str, help="1SP bin file")
    parser.add_argument("cmd_out_txt", type=str, help="1SP cmd file in Newton format")
    parser.add_argument("--production-fuses", nargs=5, type=str, help="Generate Production 1SP. Pass 5 hex files with svn, public hash, encryption key, global private key, disable modes.")

    args = parser.parse_args()
    bin2newton_cmd(args.src_bin, args.cmd_out_txt, args.production_fuses)
    # input_data = open(args.src_bin, 'rb').read()
    # with open(args.cmd_out_txt, 'w') as out_file:
        # out_file.write(get_cmd_1sp_header(input_size))
        # write_bin_in_newton_format(out_file, input_data)


if __name__ == "__main__":
    main()
