# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
This is a proof of concept python-systemc qspi slave model which uses 
systemc features exposed by the pybind11 baesd PythonBinder C++ model to
react on the coming systemc transactions and initiate the proper transactions
from/to the qspi systemc C++ model.

this backend is being used by "qupv3_qupv3_se_wrapper_se0_qspi-test.cc".
"""
from tlm_generic_payload import tlm_command, tlm_generic_payload
import qspi_biflow_socket
from sc_core import sc_time, sc_time_unit
import qspi_cpp_shared_vars
import argparse
import shlex
from queue import Queue
import array
import sys
from enum import Enum, auto

class qspi_OPCODE(Enum):
    QSPI_READ = 0,
    QSPI_WRITE = 1,
    QSPI_WRITE_READ = 2,
    QSPI_DMA = 3

# Create a table with integer addresses and values


class FLASH_OPCODE(Enum):
    RESET_ENABLE_CMD = 0x66
    RESET_CMD = 0x99
    READ_STATUS_CMD = 0x05
    READ_STATUS_2_CMD = 0x3F
    WRITE_STATUS_2_CMD = 0x3E
    WRITE_ENABLE_CMD = 0x06
    WRITE_DISABLE_CMD = 0x04
    READ_ID_CMD = 0x9F #159
    READ_SFDP_CMD = 0x5A #90
    READ_CFG1_CMD = 0x35
    READ_FLAG_STATUS_CMD = 0x70
    READ_SECURITY_CMD = 0x2B
    WRITE_STATUS_CMD = 0x01
    ENTER_4B_ADDR_CMD = 0xB7
    READ_CFG_REG_CMD = 0x15
    READ_ENHANCED_VOL_CFG_CMD = 0x65 #101
    # WRITE_CFG_CMD = 0x71
    ENTER_DPD = 0xB9 #185
    EXIT_DPD = 0xAB #171
    ISSUE_INSTR_35_CMD = 0x35
    ISSUE_INSTR_38_CMD = 0x38
    WRITE_ENHANCED_VOL_CFG_CMD = 0x61 #97
    WRITE_OCTAL_EN_STATUS_2_CMD = 0x31
    CLEAR_OCTAL_EN_STATUS_2_CMD = 0x3E
    ENABLE_8S_8S_8S_MODE_SEQ_CMD = 0xE8
    CLEAR_ERR_REGS = 0xB6
    CLEAR_FLAG_STATUS_REG = 0x50
    # READ_CFG2_CMD = 0x71
    WRITE_CFG2_CMD = 0x72
    READ_VOLATILE_CFG_CMD = 0x85
    WRITE_VOLATILE_CFG_CMD = 0x81 #129
    READ = 0x03
    READ4= 0x13
    FAST_READ4 = 0x0C
    OUTPUT_FAST_READ4 = 0x6C
    QIOR4 = 0xEC #236
    ERASE4_SECTOR = 0xDC,
    ERASE4_4K = 0x21,
    PP4 = 0x12,

import binascii

def load_binary(file_path):
    with open(file_path, 'rb') as f:
        content = f.read()
    return content

def read_data_at_address(binary_content, address, length):
    if address < 0 or address + length > len(binary_content):
        raise ValueError("Requested address and length are out of bounds")
    data = binary_content[address:address+length]
    return data

def format_data_as_int_list(data):
    return [byte for byte in data]


# def overwrite_binary_content(binary_content, address, values):
#     if address < 0 or address + len(values) > len(binary_content):
#         raise ValueError("Requested address and values are out of bounds")
#     binary_content = bytearray(binary_content)
#     binary_content[address:address+len(values)] = bytes(values)
#     return binary_content

# binary_content = load_binary('/local/mnt/workspace/personnal/SAIL_SOURCES/sail_proc/BSP/sailjtagprogrammer/singleimage/singleimage_ospi.bin') # SECTOR_SIZE_IN_BYTES="4096" filename="singleimage_ospi.bin" start_byte_hex="0x0"
binary_content = load_binary('/local/mnt/workspace/personnal/LAST_SAIL_SOURCES/sail_proc/BSP/sailjtagprogrammer/singleimage/singleimage.bin') #path to binary


address_map_w = {
    0: [0],
}
SFDP_data = [0x53, 0x46, 0x44, 0x50, 0x06, 0x01, 0x01, 0xff,
    0x00, 0x06, 0x01, 0x10, 0x30, 0x00, 0x00, 0xff,
    0x84, 0x00, 0x01, 0x02, 0x80, 0x00, 0x00, 0xff,
    0x03, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xe5, 0x20, 0x8a, 0xff, 0xff, 0xff, 0xff, 0x0f,
    0x29, 0xeb, 0x27, 0x6B, 0x27, 0x3b, 0x27, 0xbb,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x27, 0xbb,
    0xff, 0xff, 0x29, 0xeb, 0x0c, 0x20, 0x10, 0xd8,
    0x0f, 0x52, 0x00, 0x00, 0x24, 0x4A, 0x99, 0x00,
    0x8b, 0x8e, 0x03, 0xd4, 0xac, 0x01, 0x27, 0x38,
    0x7a, 0x75, 0x7a, 0x75, 0xfb, 0xbd, 0xd5, 0x5c,
    0x4A, 0x0F, 0x82, 0xff, 0x81, 0xfd, 0x1e, 0x36,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xe7, 0xff, 0xff, 0x21, 0xdc, 0x5c, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]

#   OTA_IN_PROGRESS = 0x0,                        /* Flashing to a B partition initiated. Don’t use B for normal boot */
#   OTA_UPDATE_START    = 0x1,                    /* OTA about to start update GPT SW1 */
#   OTA_BOOTING  = 0x2,                           /* OTA boot in progress */ //->in xbl if state update start change state to OTA booting update retry cnt
#   OTA_ROLLBACK = 0x3,                           /* OTA boot failed. Rollback in progress */
#   OTA_DISABLED = 0x4,                           /* OTA not initiated. Regular boot */
#   OTA_DONE = 0x5,                               /* OTA state with no 1+1 redundancy */
#   OTA_REBOOT_CORRUPTED = 0x55,                  /* OTA Reboot Corrupted Incase all state are Invalid*/
#   OTA_INVALID = 0xFF                            /* Invalid OTA State */
# Boot Flow: Initialize XBL -> Check OTA partitions datas from NOR Flahs (A/B) -> Check OTA status from paritions
# The OTA datas are like that {OTA_STATE, CRC}, let's put 0x4 by default which is disabled and the CRC for this value is 0xFB
OTA_data_partA = [0x4, 0xFB, 0x4, 0xFB, 0x04, 0xFB, 0x04, 0xFB, 0x04, 0xFB, 0x04, 0xFB, 0x04, 0xFB, 0x04, 0xFB,
            0x0, 0x0, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x0, 0x0, 0x0, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

OTA_data_partB = [0x4, 0xFB, 0x4, 0xFB, 0x04, 0xFB, 0x04, 0xFB, 0x04, 0xFB, 0x04, 0xFB, 0x04, 0xFB, 0x04, 0xFB,
            0x0, 0x0, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x0, 0x0, 0x0, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

#NOR-FLASH REGISTERS:
VOLATILE_CFG_CMD = 0xab #READ opcode 0x85 WRITE opcode 0x81
#to do WRITE_ENABLE_CMD  # 0x06 DMA - only opcode qemu case WREN: s->write_enable = true;
STATUS_REGISTER = 0xce #READ opcode 0x05
ENHANCED_VOL_CFG_CMD = 0x7b  #READ opcode 0x65 WRITE opcode 0x61

dma_indx = 0

def log(msg: str) -> None:
    if args.debug:
        print(f"[python] {msg}")


def parse_args(args_str: str) -> argparse.Namespace:
    """parse module arguments"""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--debug",
        help="enable debug messages",
        action="store_true",
    )
    return parser.parse_args(shlex.split(args_str))


args = parse_args(qspi_cpp_shared_vars.module_args)

def decode(id: int, trans: tlm_generic_payload, delay: sc_time) -> None:
    global dma_indx
    global VOLATILE_CFG_CMD
    global STATUS_REGISTER
    global ENHANCED_VOL_CFG_CMD
    txn = tlm_generic_payload()
    log(f"Flash:decode, txn")
    tx_len = trans.get_data_length()
    log(f"Flash:decode, tx_len: {tx_len}")
    address = trans.get_address()
    log(f"Flash:decode, address: {address:#x}")
    opcode = trans.get_command().value
    log(f"Flash:decode, opcode: {opcode:#x}")
    # reset the data length in order to read cmd data otherwise it will read random datas or caused segfault with very big length
    trans.set_data_length(1)
    fragment = array.array('B', memoryview(trans.get_data()))
    #fragment= fragment[0]
    if opcode == FLASH_OPCODE.READ_SFDP_CMD.value:
        print("Handling READ_SFDP_CMD")  # 0x5A -- so far always being sent with an address as a read request with len
        # Add 
        txn.set_command(tlm_command.TLM_WRITE_COMMAND)
        qspi_biflow_socket.set_default_txn(txn)
        txn.set_data_length(tx_len)
        data = format_data_as_int_list(read_data_at_address(SFDP_data, address, tx_len))
        if(tx_len > len(data)):
                raise RuntimeError(
            f"command: {trans.get_command()} requested_len > len(data[requested_offset])"
        )
        i = 0
        while i < tx_len:
            log(f"sending to master: {data[i]:#x}")
            qspi_biflow_socket.enqueue(data[i])
            i = i + 1
    elif opcode == FLASH_OPCODE.READ_ID_CMD.value:
        print("Handling READ_ID_CMD")  # 0x9F only opcode request data
        txn.set_command(tlm_command.TLM_WRITE_COMMAND)
        qspi_biflow_socket.set_default_txn(txn)
        txn.set_data_length(tx_len)
        data =[0x20, 0xBB, 0x19, 0x00] # device id 
        if(tx_len > len(data)):
                raise RuntimeError(
            f"command: {trans.get_command()} requested_len > len(data[requested_offset])"
        )
        i = 0
        while i < tx_len:
            log(f"sending to master: {data[i]:#x}")
            qspi_biflow_socket.enqueue(data[i])
            i = i + 1
    elif opcode == FLASH_OPCODE.READ_CFG1_CMD.value:
        print("Handling READ_CFG1_CMD")  # 0x35 with address as a read request with len
        # Add
    elif opcode == FLASH_OPCODE.RESET_ENABLE_CMD.value:
        print("Handling RESET_ENABLE_CMD")  # 0x66 DMA - only opcode
        # Add 
    elif opcode == FLASH_OPCODE.RESET_CMD.value:
        print("Handling RESET_CMD")  # 0x99 DMA - only opcode
        # Add  
    elif opcode == FLASH_OPCODE.WRITE_ENABLE_CMD.value:
        print("Handling WRITE_ENABLE_CMD")  # 0x06 DMA - only opcode STATUS_REGISTER(bit 7: write enable (0)/disable(1))
        STATUS_REGISTER = STATUS_REGISTER & 0x7F
        # Add
    elif opcode == FLASH_OPCODE.WRITE_DISABLE_CMD.value:
        print("Handling WRITE_DISABLE_CMD")  # 0x04 DMA - only opcode STATUS_REGISTER(bit 7: write enable (0)/disable(1))
        STATUS_REGISTER = STATUS_REGISTER & 0xFF
        # Add 
    elif opcode == FLASH_OPCODE.READ_STATUS_CMD.value:
        print("Handling READ_STATUS_CMD")  # 0x05 only opcode
        txn.set_command(tlm_command.TLM_WRITE_COMMAND)
        qspi_biflow_socket.set_default_txn(txn)
        txn.set_data_length(tx_len)
        data = STATUS_REGISTER
        log(f"sending to master: {data:#x}")
        qspi_biflow_socket.enqueue(data)
    elif opcode == FLASH_OPCODE.READ_STATUS_2_CMD.value:
        print("Handling READ_STATUS_2_CMD")  # 0x3F
        # Add 
    elif opcode == FLASH_OPCODE.WRITE_STATUS_2_CMD.value:
        print("Handling WRITE_STATUS_2_CMD")  # 0x3E
        # Add 
    elif opcode == FLASH_OPCODE.READ_FLAG_STATUS_CMD.value: 
        print("Handling READ_FLAG_STATUS_CMD")  # 0x70
        txn.set_command(tlm_command.TLM_WRITE_COMMAND)
        qspi_biflow_socket.set_default_txn(txn)
        txn.set_data_length(tx_len)
        data = format_data_as_int_list(read_data_at_address(binary_content, address, tx_len))
        if(tx_len > len(data)):
                raise RuntimeError(
            f"command: {trans.get_command()} requested_len > len(data[requested_offset])"
        )
        i = 0
        while i < tx_len:
            log(f"sending to master: {data[i]:#x}")
            qspi_biflow_socket.enqueue(data[i])
            i = i + 1
        # Add 
    elif opcode == FLASH_OPCODE.READ_SECURITY_CMD.value:
        print("Handling READ_SECURITY_CMD")  # 0x2B
        # Add 
    elif opcode == FLASH_OPCODE.WRITE_STATUS_CMD.value:
        print("Handling WRITE_STATUS_CMD")  # 0x01
        # Add 
    elif opcode == FLASH_OPCODE.ENTER_4B_ADDR_CMD.value:
        print("Handling ENTER_4B_ADDR_CMD")  # 0xB7
        # Add 
    elif opcode == FLASH_OPCODE.READ_CFG_REG_CMD.value:
        print("Handling READ_CFG_REG_CMD")  # 0x15
        # Add 
    elif opcode == FLASH_OPCODE.READ_ENHANCED_VOL_CFG_CMD.value:
        print("Handling READ_ENHANCED_VOL_CFG_CMD")  # 0x65 page 30 Enhanced Volatile Configuration Register
        txn.set_command(tlm_command.TLM_WRITE_COMMAND)
        qspi_biflow_socket.set_default_txn(txn)
        txn.set_data_length(tx_len)
        data = ENHANCED_VOL_CFG_CMD
        log(f"sending to master: {data:#x}")
        qspi_biflow_socket.enqueue(data)

        # Add 
    # elif opcode == FLASH_OPCODE.WRITE_CFG_CMD.value:
    #     print("Handling WRITE_CFG_CMD")  # 0x71
    #     # Add 
    elif opcode == FLASH_OPCODE.ENTER_DPD.value:
        print("Handling ENTER_DPD")  # 0xB9 DMA - only opcode
        # Add 
    elif opcode == FLASH_OPCODE.EXIT_DPD.value:
        print("Handling EXIT_DPD")  # 0xAB
        # Add 
    elif opcode == FLASH_OPCODE.ISSUE_INSTR_35_CMD.value:
        print("Handling ISSUE_INSTR_35_CMD")  # 0x35
        # Add 
    elif opcode == FLASH_OPCODE.ISSUE_INSTR_38_CMD.value:
        print("Handling ISSUE_INSTR_38_CMD")  # 0x38
        # Add 
    elif opcode == FLASH_OPCODE.WRITE_ENHANCED_VOL_CFG_CMD.value:
        print("Handling WRITE_ENHANCED_VOL_CFG_CMD")  # 0x61
        data = array.array('B', memoryview(trans.get_data()))
        ENHANCED_VOL_CFG_CMD = data[0]
        log(f"SW writes value to ENHANCED_VOL_CFG_CMD : {ENHANCED_VOL_CFG_CMD:x}") 
    elif opcode == FLASH_OPCODE.WRITE_OCTAL_EN_STATUS_2_CMD.value:
        print("Handling WRITE_OCTAL_EN_STATUS_2_CMD")  # 0x31
        # Add 
    elif opcode == FLASH_OPCODE.CLEAR_OCTAL_EN_STATUS_2_CMD.value:
        print("Handling CLEAR_OCTAL_EN_STATUS_2_CMD")  # 0x3E
        # Add 
    elif opcode == FLASH_OPCODE.ENABLE_8S_8S_8S_MODE_SEQ_CMD.value:
        print("Handling ENABLE_8S_8S_8S_MODE_SEQ_CMD")  # 0xE8
        # Add 
    elif opcode == FLASH_OPCODE.CLEAR_ERR_REGS.value:
        print("Handling CLEAR_ERR_REGS")  # 0xB6
        # Add 
    elif opcode == FLASH_OPCODE.CLEAR_FLAG_STATUS_REG.value:
        print("Handling CLEAR_FLAG_STATUS_REG")  # 0x50
        # Add 
    # elif opcode == FLASH_OPCODE.READ_CFG2_CMD.value:
    #     print("Handling READ_CFG2_CMD")  # 0x71
    #     # Add 
    elif opcode == FLASH_OPCODE.WRITE_CFG2_CMD.value:
        print("Handling WRITE_CFG2_CMD")  # 0x72
        # Add 
    elif opcode == FLASH_OPCODE.READ_VOLATILE_CFG_CMD.value:
        print("Handling READ_VOLATILE_CFG_CMD")  # 0x85
        txn.set_command(tlm_command.TLM_WRITE_COMMAND)
        qspi_biflow_socket.set_default_txn(txn)
        txn.set_data_length(tx_len)
        data = VOLATILE_CFG_CMD
        log(f"sending to master: {data:#x}")
        qspi_biflow_socket.enqueue(data)
    elif opcode == FLASH_OPCODE.WRITE_VOLATILE_CFG_CMD.value:
        print("Handling WRITE_VOLATILE_CFG_CMD")  # 0x81 WRITE OPCODE - only one byte
        data = array.array('B', memoryview(trans.get_data()))
        VOLATILE_CFG_CMD = data[0]
        log(f"SW writes value to VOLATILE_CFG_CMD : {VOLATILE_CFG_CMD:x}")
    elif opcode == FLASH_OPCODE.READ.value:
        print("Handling READ") #03
        log(f"fragment: {fragment}")
        log(f"fragment[0]: {fragment[0]}")
        txn.set_command(tlm_command.TLM_WRITE_COMMAND)
        qspi_biflow_socket.set_default_txn(txn)
        data = format_data_as_int_list(read_data_at_address(binary_content, address, tx_len))
        # log(f", dma_indx: {dma_indx}")
        txn.set_data_length(tx_len)
        # dma_indx= dma_indx+tx_len
        if(fragment[0]==0):
            dma_indx=0
        # log(f", dma_indx: {dma_indx}")
        log(f", data_len: {len(data)}")
        log(f", requested_len: {tx_len}")
        if(tx_len > len(data)):
                raise RuntimeError(
            f"command: {trans.get_command()} requested_len > len(data[requested_offset])")
        i = 0
        while i < tx_len:
            log(f"DMA sending to master: {data[i]:#x}")
            qspi_biflow_socket.enqueue(data[i])
            i = i + 1
    elif opcode == FLASH_OPCODE.READ4.value:
        print("Handling READ4") #13 DMA - with address as a read request with len
        log(f"fragment: {fragment}")
        log(f"fragment[0]: {fragment[0]}")
        txn.set_command(tlm_command.TLM_WRITE_COMMAND)
        qspi_biflow_socket.set_default_txn(txn)
        data = format_data_as_int_list(read_data_at_address(binary_content, address + dma_indx, tx_len))
        log(f", dma_indx: {dma_indx}")
        txn.set_data_length(tx_len)
        dma_indx= dma_indx+tx_len
        if(fragment[0]==0):
            dma_indx=0
        log(f", dma_indx: {dma_indx}")
        log(f", data_len: {len(data)}")
        log(f", requested_len: {tx_len}")
        if(tx_len > len(data)):
                raise RuntimeError(
            f"command: {trans.get_command()} requested_len > len(data[requested_offset])")
        i = 0
        while i < tx_len:
            log(f"DMA sending to master: {data[i]:#x}")
            qspi_biflow_socket.enqueue(data[i])
            i = i + 1
    elif opcode == FLASH_OPCODE.FAST_READ4.value:
        print("Handling FAST_READ4") #0c DMA - with address and len
        # Add
    elif opcode == FLASH_OPCODE.OUTPUT_FAST_READ4.value:
        print("Handling OUTPUT_FAST_READ4") #6C DMA - with address and len - opcode 6C should be use when we are in mode 1-1-4
        log(f"fragment: {fragment}")
        log(f"fragment[0]: {fragment[0]}")
        txn.set_command(tlm_command.TLM_WRITE_COMMAND)
        qspi_biflow_socket.set_default_txn(txn)
        if(address==0x1fff000):
            address=0x1c33000
        data = format_data_as_int_list(read_data_at_address(binary_content, address + dma_indx, tx_len))
        if(address == 0xDE3000):
            data = OTA_data_partA
        if(address==0xDE9000):
            data = OTA_data_partB
        log(f", dma_indx: {dma_indx}")
        txn.set_data_length(tx_len)
        dma_indx= dma_indx+tx_len
        if(fragment[0]==0):
            dma_indx=0
        log(f", dma_indx: {dma_indx}")
        log(f", data_len: {len(data)}")
        log(f", requested_len: {tx_len}")
        if(tx_len > len(data)):
                raise RuntimeError(
            f"command: {trans.get_command()} requested_len > len(data[requested_offset])")
        i = 0
        while i < tx_len:
            log(f"DMA sending to master: {data[i]:#x}")
            qspi_biflow_socket.enqueue(data[i])
            i = i + 1
    elif opcode == FLASH_OPCODE.QIOR4.value:
        print("Handling QIOR4") #EC DMA - with address and len - opcode EC should be use when we are in mode 4-4-4
        log(f"fragment: {fragment}")
        log(f"fragment[0]: {fragment[0]}")
        txn.set_command(tlm_command.TLM_WRITE_COMMAND)
        qspi_biflow_socket.set_default_txn(txn)
        if(address==0x1fff000):
            address=0x1c33000
        data = format_data_as_int_list(read_data_at_address(binary_content, address + dma_indx, tx_len))
        if(address==0xDE3000):
            data = OTA_data_partA
        log(f", dma_indx: {dma_indx}")
        txn.set_data_length(tx_len)
        dma_indx= dma_indx+tx_len
        if(fragment[0]==0):
            dma_indx=0
        log(f", dma_indx: {dma_indx}")
        log(f", data_len: {len(data)}")
        log(f", requested_len: {tx_len}")
        if(tx_len > len(data)):
                raise RuntimeError(
            f"command: {trans.get_command()} requested_len > len(data[requested_offset])")
        i = 0
        while i < tx_len:
            log(f"DMA sending to master: {data[i]:#x}")
            qspi_biflow_socket.enqueue(data[i])
            i = i + 1
    elif opcode == FLASH_OPCODE.ERASE4_SECTOR.value:
        print("Handling ERASE4_SECTOR") #DC opcode and address only
    elif opcode == FLASH_OPCODE.ERASE4_4K.value: # Add
        print("Handling ERASE4_4K") #21 opcode and address only
        # Add
    elif opcode == FLASH_OPCODE.PP4.value:
        print("Handling PP4") #12 DMA - with address and len - is it always dma? 
        #WRIE TO THE FLASH 128 byte
    else:
        print("Unknown opcode")

def set_send_limit_to_inf() -> None:
    """send the ctrl struct with cmd set to INFINITE to qspi"""
    try:
        qspi_biflow_socket.can_receive_any()
    except SystemExit:
        return
    except Exception as e:
        raise Exception(f"set_send_limit_to_inf error: {e}")


def before_end_of_elaboration() -> None:
    """called by the C++ systemc kernel before end of elaboration"""
    try:
        set_send_limit_to_inf()
    except SystemExit:
        return
    except Exception as e:
        raise Exception(f"before_end_of_elaboration error: {e}")


# Entry point of the script, this function will be called from the PythonBinder module.
def bf_b_transport(trans: tlm_generic_payload, delay: sc_time) -> None:
    log(f"transaction: {trans}")
    log(f"transaction id: {id}")
    log(f"tlm_command: {trans.get_command().value}")
    """Make sure the trans and slave addresses are identical"""
    decode(id, trans, delay)
