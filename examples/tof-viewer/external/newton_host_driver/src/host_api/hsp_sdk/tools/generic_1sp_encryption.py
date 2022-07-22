from ctypes import *
import argparse
from Crypto.Cipher import AES
import hashlib
import struct

KdfKeyAllowed = 1024
AesDecryptAllowed = 4
ENTRY_POINT = 128

def process_args(src_bin, svn_txt, encr_txt, kdf_val_txt, iv_val_txt, out_bin):
    with open(src_bin, 'rb') as src_bin_file:
        sp_image = src_bin_file.read()
        src_bin_file.close()

    with open(svn_txt, 'r') as svn_txt_file:
        svn_val = svn_txt_file.read()
        svn_txt_file.close()

    with open(encr_txt, 'r') as encr_txt_file:
        ency_key = encr_txt_file.read()
        encr_txt_file.close()

    with open(kdf_val_txt, 'r') as kdf_val_txt_file:
        kdf_val = kdf_val_txt_file.read()
        kdf_val_txt_file.close()

    with open(iv_val_txt, 'rb') as iv_val_txt_file:
        iv_val = iv_val_txt_file.read()
        iv_val_txt_file.close()

    ency_key = bytes.fromhex(ency_key)
    kdf_val = bytes.fromhex(kdf_val)
    svn_val = int(svn_val, 16)

    m = hashlib.sha256()

    padding_for_kdf_val = 32 - len(kdf_val)
    padding_bytes = bytes(padding_for_kdf_val)
    kdf_val = padding_bytes + kdf_val

    # fixing endianness
    kdf_val_hex = kdf_val.hex()
    kdf_val_hex = kdf_val_hex[::-1]
    r_str = ''
    for i in range(0, len(kdf_val_hex), 2):
        cand = kdf_val_hex[i:i + 2]
        r_str += cand[::-1]
    kdf_val = bytes.fromhex(r_str)

    attributes = (KdfKeyAllowed | AesDecryptAllowed).to_bytes(3, byteorder='little')

    print(f"ency_key is {ency_key.hex()}")
    print(f"kdf_val is {kdf_val.hex()}")
    print(f"attributes is {attributes.hex()}")

    assert(len(ency_key) == 32)
    assert(len(kdf_val) == 32)
    assert (len(attributes) == 3)

    #derive the key
    m.update(kdf_val)
    m.update(ency_key)
    m.update(attributes)

    key = m.digest()
    print(f"key len is {len(key)}")
    print(f"key is {key.hex()}")
    print(f"len of sp_image is {len(sp_image)}")

    if len(sp_image) % 16:
        padding = bytes((16 - (len(sp_image) % 16)))
        sp_image = sp_image + padding

    print(f"type of sp_image is {type(sp_image)}")
    print(f"len of sp_image is {len(sp_image)}")

    plaintext_m = hashlib.sha256()
    plaintext_m.update(sp_image)
    plaintexthash = plaintext_m.digest()

    print(f"iv_val is {iv_val.hex()}")

    #######################
    # fixing endianness
    iv_val_hex = iv_val.hex()
    r_str = ''
    for i in range(0, len(iv_val_hex), 8):
        cand = iv_val_hex[i:i + 8]
        reverse_cand = cand[::-1]
        for j in range(0,len(cand),2):
            sub_str = reverse_cand[j:j+2]
            r_str += sub_str[::-1]
    iv_val = bytes.fromhex(r_str)

    print(f"iv_val is {iv_val.hex()}")
    print(f"iv_val len is {len(iv_val)}")
    ######################

    cipher = AES.new(key, AES.MODE_CBC, iv=iv_val)
    ciphertext = cipher.encrypt(sp_image)

    headerVersion = 0
    OneSPVersion = svn_val
    Size = len(sp_image)
    Entry = ENTRY_POINT # size of header
    CodePlaintextHash = plaintexthash
    print(f"CodePlaintextHash in hex is {CodePlaintextHash.hex()}")
    CodePlaintextHashArr = [x for x in CodePlaintextHash]


    out_data = bytearray()
    out_data.extend(struct.pack('<I', headerVersion))
    out_data.extend(struct.pack('<I', OneSPVersion))
    out_data.extend(struct.pack('<I', Size))
    out_data.extend(struct.pack('<I', Entry))
    out_data.extend(struct.pack('<32B', *CodePlaintextHashArr))
    headerPadding = 128 - 32 - 16 # subtracting all preceding fields size
    paddingArr = [0]*headerPadding
    headerPaddingStr = str(headerPadding)
    out_data.extend(struct.pack('<'+headerPaddingStr+'B', *paddingArr))

    print(f"printing the first 50 of the ciphertext")
    print(f"{ciphertext[0:50].hex()}")

    with open(out_bin, 'wb') as out_bin_file:
        out_bin_file.write(out_data)
        out_bin_file.write(ciphertext)
    out_bin_file.close()



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("src_bin", type=str, help="file path")
    parser.add_argument("svn_txt", type=str, help="svn txt file")
    parser.add_argument("encr_txt", type=str, help="encryption txt file")
    parser.add_argument("kdf_val_txt", type=str, help="kdf value txt file")
    parser.add_argument("iv_val_txt", type=str, help="initial vector txt file")
    parser.add_argument("out_bin", type=str, help="output file path")

    args = parser.parse_args()
    process_args(args.src_bin, args.svn_txt, args.encr_txt, args.kdf_val_txt, args.iv_val_txt, args.out_bin)


if __name__ == "__main__":
    main()