#!/usr/bin/env python

# ============================
# crypto functions

from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes

import hashlib
from ecdsa import SigningKey, VerifyingKey
from ecdsa.util import sigencode_der, sigdecode_der
from ecdsa import NIST256p

import binascii

def encryptData(binaryData, width=0, keyFilePath = '', ivFilePath = ''):
    ciphertext = None

    print(f"About to encrypt, binaryData provided is {binaryData.hex()}")
    if width == 0:
        print("encrypting data")
        # standard data, standard encryption method
        ciphertext = encryptDataUsingAes(binaryData, keyFilePath, ivFilePath)
    elif width == 11:
        print("encrypting 11 bit wide data")
        # data is 11 bit wide, additional processing required
    elif width == 24:
        print("encrypting 24 bit wide data")
        #data is 24 bit wide, additional processing required

    binaryData = ciphertext
    print(f"Finished encryption, ciphertext is {ciphertext.hex()}")

    return ciphertext

def signData(binaryData, width = 0, filepath = '',  outputFileName=" "):
    signature = bytearray()

    if width == 0:
        print("signing data")
        # standard data, standard encryption method
        print(f"binary data size {len(binaryData)}")
        print(f"binary data first 64 {binaryData[:64].hex()}")
        print(f"binary data last 64 {binaryData[len(binaryData)-64:].hex()}")
        # print(f"binary data is {binaryData.hex()}")
        with open ("./signed_data.bin", "wb") as fbin:
            fbin.write(binaryData);
        fbin.close()
        signature = signUsingECC(binaryData, filepath, outputFileName)
    elif width == 11:
        print("signing 11 bit wide data")
        print("signing data")
        # standard data, standard encryption method
        print(f"binary data size {len(binaryData)}")
        print(f"binary data first 64 {binaryData[:64].hex()}")
        print(f"binary data last 64 {binaryData[len(binaryData)-64:].hex()}")
        print(f"binary data is {binaryData.hex()}")
        with open ("./signed_data.bin", "wb") as fbin:
            fbin.write(binaryData);
        fbin.close()
        signature = signUsingECC(binaryData, filepath, outputFileName)
        # data is 11 bit wide, additional processing required
    elif width == 24:
        print("signing 24 bit wide data")
        #data is 24 bit wide, additional processing required
    
    return signature

def encryptDataUsingAes(binaryData, keyFilePath='', ivFilePath=''):
    # assuming keys and iv
    keyFilePath, ivFilePath = '', '' # temporary force here
    if keyFilePath == '':
        # key = [0xc7, 0x9d, 0xc2, 0xca, 0x34, 0xc7, 0x43, 0xbb, 0x22, 0xf5, 0xa4, 0x82, 0x70, 0x3b, 0x02, 0xeb] # base example
        # key = [0x22, 0x98, 0x65, 0x8a, 0x16, 0xb0, 0x7b, 0x5f, 0x8e, 0xe3, 0x8b, 0xbd, 0xdd, 0x6a, 0xf6, 0xe1] # test state
        key = [0x6d, 0x11, 0x93, 0x20, 0x6c, 0x9b, 0xf3, 0x5a, 0x03, 0x41, 0x9b, 0x64, 0xb5, 0xc9, 0xf0, 0xc0]
    else:
        with open(keyFilePath, 'rb') as fp:
            key = fp.read()
        fp.close()

    # print(f"key type is {type(key)}, and key is {key}")
    if ivFilePath == '':
        iv = [0xc1, 0x61, 0x42, 0xfa, 0x94, 0x99, 0x0e, 0x80, 0x6b, 0xaf, 0x9a, 0xbb, 0xd5, 0x01, 0x37, 0xc8]
    else:
        with open(ivFilePath, 'rb') as fp:
            iv = fp.read()
        fp.close()

    key = bytes(key)
    iv = bytes(iv)
    key_hex = key.hex()
    iv_hex = iv.hex()
    
    data = bytes(binaryData)
    data_hex = data.hex()
    
    cipher = AES.new(key, AES.MODE_CBC, iv=iv)
    ciphertext = cipher.encrypt(data)
    return ciphertext

def signUsingECC(binaryData, privateKeyFilePath = '', outputFileName=" "):
    if privateKeyFilePath == '':
        # privateKeyFilePath = "C:\\HSP_fw\\push_branch\\test\\keys\\TestKey1.pem"
        privateKeyFilePath = "..\\..\\test\\keys\\TestKey1.pem"
        print("Signing key not provided")
        exit(0)
    
    print(f"key being used to sign is {privateKeyFilePath}")

    with open(privateKeyFilePath) as f:
        sk = SigningKey.from_pem(f.read(), hashlib.sha256)

    message = bytes(binaryData)

    with open("./data.bin", 'wb') as d:
        d.write(binaryData)
    d.close()

    m = hashlib.sha256()
    m.update(message)
    digest = m.digest()
    print(f"hash is {digest.hex()}")
    # print(f"hash digest.hex() type is {type(digest.hex())}")
    print(f"hash is {digest}")
    # print(f"hash digest type is {type(digest)}")
    hexstring = digest.hex()
    digest_base64 = binascii.b2a_base64(bytes.fromhex(hexstring))
    print(f"digest_base64 is {digest_base64}")
    # print(f"digest_base64 type is {type(digest_base64)}")

    with open("./" + outputFileName + ".b64", "wb") as digest_file:
        digest_file.write(digest_base64)
    digest_file.close()


    crc_value = hex(binascii.crc32(message))
    print(f"crc value is {crc_value}")

    signature = sk.sign(message)
    signature = bytearray(signature)
    # print(f"Finished signing, returning signature. signature len is {len(signature)}, type is {type(signature)}\nsignature is {signature.hex()}")
    print(f"signing key is {sk.to_string().hex()}")
    first_half = signature[:len(signature)//2]
    second_half = signature[len(signature)//2:]
    print(f"first half is {first_half}, second half is {second_half}")
    first_half = first_half[::-1]
    second_half = second_half[::-1]
    reverse_signature = first_half + second_half
    # reverse_signature = signature[::-1]
    print(f"reverse_sig is {reverse_signature.hex()}")
    # return signature
    return reverse_signature
