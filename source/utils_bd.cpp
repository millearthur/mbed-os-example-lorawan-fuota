#include "utils_bd.h"

#include "mbed_trace.h"
#define TRACE_GROUP "U_BD"

#define SIZE_BD_ERASE 0x210

// ------------------------------------------------------------- //
// ------------------------------------------------------------- //
void remove_data_bd(BlockDevice *bd,int slot0, int slot1){
   
    uint8_t zeroes[SIZE_BD_ERASE] = {0};             //  erase size 0x210 == 528
    int addr_start;
    int addr_end;
    uint8_t nbRep = 0;

    if(slot0 == 1){
        //ERASE FIRST SLOT (SLOT 0)
        addr_start = 528;
        addr_end = 262944;
        nbRep = (addr_end-addr_start) / 528;    //  nbRep to get from addr_start to addr_end
        for (size_t i = 0; i < nbRep; i++)
        {
            tr_debug("Erase %d bytes from : %x \n",SIZE_BD_ERASE,(addr_start + (i*SIZE_BD_ERASE)));
            bd->program(zeroes, (addr_start + (i*SIZE_BD_ERASE)), SIZE_BD_ERASE);
        }
        tr_info("Erase SLOT 0 DONE\n");
    }
    
    if(slot0 == 1){
        //ERASE SLOT 1
        addr_start = 0x40320;
        addr_end = 0x80430;
        nbRep = (addr_end-addr_start) / 528;    //  nbRep to get from addr_start to addr_end
        for (size_t i = 0; i < nbRep; i++)
        {
            tr_debug("Erase %d bytes from : %x \n",SIZE_BD_ERASE,(addr_start + (i*SIZE_BD_ERASE)));
            bd->program(zeroes, (addr_start + (i*SIZE_BD_ERASE)), SIZE_BD_ERASE);
        }
        tr_info("Erase SLOT 1 DONE\n");
    }

}

// ------------------------------------------------------------- //
// ------------------------------------------------------------- //
    
void program_data_bd(BlockDevice *bd){
    
     FragmentationBlockDeviceWrapper block_device = FragmentationBlockDeviceWrapper(bd);
    block_device.init();

    //Fill FIRST SLOT (SLOT 0) with data
    //  arithmetic data
    //short w/o signature 
    //uint8_t data[] ={ 0xf9, 0x89, 0xed, 0x7c, 0x10, 0x07, 0xf0, 0x70, 0x44, 0xc1, 0x08, 0x4b, 0xd8, 0xa2, 0x00, 0x00, 0x24, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x80, 0x00, 0x08, 0x42, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0xbf, 0xfd, 0xb5, 0x81, 0xdf, 0x87, 0x31, 0xd2, 0xcb, 0xda, 0x8e, 0x88, 0xf4, 0xfb, 0xdb, 0x4a, 0xa7, 0xc3, 0x12, 0xad, 0xb8, 0x92, 0x99, 0x80, 0x53, 0x38, 0xc1, 0x7d, 0x4f, 0xcb, 0xaf, 0x4b, 0x4c, 0x9d, 0x21, 0x40, 0x75, 0xb8, 0xa7, 0x3b, 0xa4, 0xac, 0x45, 0x3e, 0xd3, 0x52, 0xe3, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x70, 0xa4, 0x66, 0x0d, 0x26, 0x5b, 0x7d, 0xc8, 0xf6, 0x9b, 0xf8};
    //long with signature 
    uint8_t data[] ={ 0xf9, 0x89, 0xed, 0x7c, 0x10, 0x07, 0xf0, 0x70, 0x44, 0xc1, 0x08, 0x4b, 0xd8, 0xa2, 0x00, 0x00, 0x24, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x80, 0x00, 0x08, 0x42, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0xbf, 0xfd, 0xb5, 0x81, 0xdf, 0x87, 0x31, 0xd2, 0xcb, 0xda, 0x8e, 0x88, 0xf4, 0xfb, 0xdb, 0x4a, 0xa7, 0xc3, 0x12, 0xad, 0xb8, 0x92, 0x99, 0x80, 0x53, 0x38, 0xc1, 0x7d, 0x4f, 0xcb, 0xaf, 0x4b, 0x4c, 0x9d, 0x21, 0x40, 0x75, 0xb8, 0xa7, 0x3b, 0xa4, 0xac, 0x45, 0x3e, 0xd3, 0x52, 0xe3, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x70, 0xa4, 0x66, 0x0d, 0x26, 0x5b, 0x7d, 0xc8, 0xf6, 0x9b, 0xf8, 0x46, 0x30, 0x44, 0x02, 0x20, 0x06, 0xad, 0x99, 0xaf, 0xdc, 0xc5, 0x6a, 0xa5, 0x42, 0x6c, 0x41, 0x34, 0xbc, 0x0e, 0x81, 0x00, 0x5b, 0xc4, 0x22, 0x3a, 0xae, 0xff, 0x2d, 0x54, 0xda, 0x70, 0x94, 0xaa, 0xbc, 0xda, 0x75, 0xaf, 0x02, 0x20, 0x00, 0xd9, 0xd5, 0xec, 0x52, 0x01, 0x30, 0x44, 0x6e, 0x1b, 0x6d, 0x77, 0x70, 0x7e, 0xdf, 0xec, 0x56, 0x86, 0x6f, 0x63, 0xf5, 0x03, 0x23, 0x72, 0xb8, 0x94, 0xcf, 0xb7, 0x9b, 0x62, 0x59, 0x22, 0x00, 0x00, 0xee, 0x33, 0x92, 0x7e, 0xe9, 0x85, 0x57, 0x7b, 0xae, 0x88, 0xfe, 0x98, 0xf5, 0xa4, 0xb5, 0x87, 0x22, 0xc3, 0xfa, 0x46, 0xb7, 0x97, 0x59, 0x5f, 0xbd, 0x1d, 0x2c, 0x18, 0x13, 0x3d, 0x42, 0x1f, 0xf7, 0x68, 0xa6, 0x60, 0x02, 0x03, 0xd8, 0x7c};
    // uint8_t data[] = { 0x99, 0x1C, 0xE5, 0xE0, 0xF4, 0x07, 0xF0, 0x78, 0x45, 0xC1, 0x08, 0x47, 0x78, 0xBE, 0x00, 0x23, 0x96, 0x02, 0x4C, 0xCF, 0xEB, 0xB0, 0x05, 0x08, 0x0F, 0xF6, 0xA8, 0x00, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x00, 0x80, 0x04, 0x04, 0x21, 0x00, 0x7E, 0x24, 0x08, 0x1F, 0x04, 0x01, 0x19, 0x00, 0x40, 0x01, 0x06, 0x10, 0x84, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x10, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x78, 0x1F, 0x40, 0x3E, 0xD0, 0x20, 0xF9, 0x0B, 0x7F, 0xD0, 0x3A, 0x75, 0x23, 0xB6, 0xF6, 0x44, 0xBD, 0xA7, 0xC4, 0xF1, 0x03, 0x8A, 0x38, 0x90, 0x8F, 0xC4, 0xEE, 0x10, 0xDC, 0x32, 0xB7, 0x77, 0x7F, 0x17, 0x5F, 0xC1, 0xF9, 0x6D, 0x4B, 0x3C, 0xDE, 0x5B, 0x7E, 0x5D, 0x18, 0x3B, 0x01, 0xD1, 0x20, 0x7F, 0x2B, 0xD1, 0xBE, 0xCD, 0xFF, 0xBD, 0xC0, 0x07, 0xE5, 0xFB, 0x2C, 0x69, 0x20, 0x87, 0xAC, 0x30, 0xE2, 0x0A, 0xEF, 0x4E, 0xFD, 0x58, 0x49, 0x14, 0x50, 0xD3, 0x3E, 0x38, 0x63, 0x3F, 0xF1, 0xBB, 0xFD, 0xD2, 0xEC, 0x85, 0xD6, 0xC9, 0x74, 0x60, 0x67, 0xA9, 0x6E, 0x86, 0xBF, 0xE1, 0xF1, 0x24, 0x1E, 0x3A, 0x2E, 0xBB, 0x6E, 0x64, 0x08, 0xA2, 0xA7, 0xCF, 0x5D, 0x2B, 0x89, 0x36, 0xAF, 0x89, 0x7D, 0xB7, 0xA5, 0x65, 0xF7, 0xFD, 0xE7, 0x94, 0x9D, 0xA8, 0x12, 0xDB, 0x19, 0xDA, 0xDB, 0xB0, 0xA5, 0x2A, 0x57, 0xB2, 0xE6, 0x14, 0x90, 0x7D, 0x0F, 0x4E, 0xD3, 0xC8, 0xA9, 0x62, 0xA4, 0x7D, 0xA0, 0x62, 0x57, 0x3D, 0x47, 0xAD, 0x47, 0xCF, 0x43, 0x9C, 0x81, 0xAF, 0x28, 0x46, 0x42, 0xE6, 0x26, 0x46, 0x35, 0xBD, 0xCA, 0xEE, 0x1E, 0x62, 0x8B, 0x49, 0xF1, 0x6B, 0x2D, 0x19, 0xE6, 0x7F, 0x0E, 0x9E, 0x0E, 0x43, 0x61, 0x70, 0x1B, 0x1E, 0xAE, 0x0D, 0x88, 0x43, 0x4F, 0xBA, 0xE4, 0x5E, 0xE8, 0x80, 0xB7, 0x1F, 0x35, 0x3E, 0x5B, 0x6A, 0x8A, 0x6B, 0x54, 0xE5, 0x74, 0xC3, 0xE2, 0xFE, 0xA0, 0x68, 0xD1, 0xD1, 0xC9, 0x6D, 0xAF, 0xC6, 0x40, 0xE7, 0xD4, 0x83, 0xD0, 0xF5, 0x2B, 0x7C, 0x44, 0xF6, 0xF0, 0xF5, 0xC2, 0x3C, 0xC2, 0x65, 0x9D, 0xD8, 0x2C, 0x53, 0x7C, 0x26, 0x6B, 0xB3, 0x06, 0x33, 0xFB, 0xF3, 0xC9, 0xF8, 0x0B, 0x6F, 0xA2, 0x09, 0xF7, 0xEC, 0xD7, 0xDE, 0x14, 0x40, 0x34, 0xC7, 0x38, 0xBC, 0x2B, 0x9F, 0x7E, 0x1F, 0x4D, 0x98, 0x8B, 0x0B, 0xC1, 0x76, 0xE9, 0xCF, 0x24, 0x98, 0x20, 0xC1, 0x03, 0x2D, 0x81, 0x83, 0x75, 0x83, 0x94, 0x33, 0x6C, 0x2A, 0xC0, 0xFA, 0xD1, 0x74, 0xA9, 0xE1, 0x0E, 0xE8, 0x66, 0x69, 0x5F, 0x3D, 0x7A, 0xDC, 0xC5, 0x78, 0xE0, 0x1F, 0x83, 0xF8, 0xA2, 0x96, 0x55, 0x3B, 0xE8, 0xCB, 0x69, 0x73, 0x1B, 0x11, 0xCC, 0x0D, 0x14, 0x34, 0x6F, 0xB7, 0x91, 0x83, 0x7A, 0x4F, 0xA9, 0x3F, 0x52, 0x03, 0x01, 0x7F, 0xFF, 0x5E, 0x7B, 0x2D, 0xD8, 0x5F, 0xC2, 0xB9, 0x53, 0x96, 0x8D, 0x51, 0x3B, 0x20, 0x1C, 0x6E, 0x08, 0x96, 0x99, 0x84, 0x67, 0x5E, 0xF7, 0x0B, 0xDD, 0xF8, 0xAE, 0x02, 0x88, 0xD8, 0xC4, 0x98, 0x19, 0xA9, 0x7D, 0x30, 0xF9, 0xC6, 0xE3, 0x90, 0x31, 0x4C, 0xDE, 0x29, 0xB0, 0xAC, 0x2D, 0xD3, 0x70, 0xAF, 0x76, 0x3B, 0xF1, 0x01, 0x08, 0x9B, 0x0C, 0xC9, 0x81, 0xB1, 0xCE, 0x39, 0xC2, 0x07, 0x46, 0x62, 0x02, 0x64, 0x7F};//{ 0x99, 0x1C, 0xE5, 0xE0, 0xF4, 0x07, 0xF0, 0x78, 0x45, 0xC1, 0x08, 0x47, 0x78, 0xBE, 0x00, 0x23, 0x96, 0x02, 0x4C, 0xCF, 0xEB, 0xB0, 0x05, 0x08, 0x0F, 0xF6, 0xA8, 0x00, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x00, 0x80, 0x04, 0x04, 0x21, 0x00, 0x7E, 0x24, 0x08, 0x1F, 0x04, 0x01, 0x19, 0x00, 0x40, 0x01, 0x06, 0x10, 0x84, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x10, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x78, 0x1F, 0x40, 0x3E, 0xD0, 0x20, 0xF9, 0x0B, 0x7F, 0xD0, 0x3A, 0x75, 0x23, 0xB6, 0xF6, 0x44, 0xBD, 0xA7, 0xC4, 0xF1, 0x03, 0x8A, 0x38, 0x90, 0x8F, 0xC4, 0xEE, 0x10, 0xDC, 0x32, 0xB7, 0x77, 0x7F, 0x17, 0x5F, 0xC1, 0xF9, 0x6D, 0x4B, 0x3C, 0xDE, 0x5B, 0x7E, 0x5D, 0x18, 0x3B, 0x01, 0xD1, 0x20, 0x7F, 0x2B, 0xD1, 0xBE, 0xCD, 0xFF, 0xBD, 0xC0, 0x07, 0xE5, 0xFB, 0x2C, 0x69, 0x20, 0x87, 0xAC, 0x30, 0xE2, 0x0A, 0xEF, 0x4E, 0xFD, 0x58, 0x49, 0x14, 0x50, 0xD3, 0x3E, 0x38, 0x63, 0x3F, 0xF1, 0xBB, 0xFD, 0xD2, 0xEC, 0x85, 0xD6, 0xC9, 0x74, 0x60, 0x67, 0xA9, 0x6E, 0x86, 0xBF, 0xE1, 0xF1, 0x24, 0x1E, 0x3A, 0x2E, 0xBB, 0x6E, 0x64, 0x08, 0xA2, 0xA7};
    //  LZ4 DATA   //uint8_t data[] = { 0x04, 0x22, 0x4D, 0x18, 0x64, 0x50, 0x08, 0x51, 0x0A, 0x00, 0x00, 0x90, 0x44, 0x44, 0x45, 0x4C, 0x54, 0x41, 0x34, 0x30, 0x00, 0x01, 0x00, 0x32, 0x01, 0xC0, 0xB4, 0x08, 0x00, 0x32, 0x5A, 0x6A, 0x00, 0x01, 0x00, 0x22, 0x04, 0xFF, 0x01, 0x00, 0x2F, 0xF8, 0x00, 0x01, 0x00, 0xFF, 0x49, 0x1F, 0x50, 0x1C, 0x00, 0x24, 0x00, 0x01, 0x00, 0x1F, 0x08, 0x64, 0x00, 0x14, 0x2F, 0x08, 0x00, 0x01, 0x00, 0xFF, 0x64, 0x2F, 0x08, 0x00, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0x24, 0x1F, 0x08, 0x58, 0x03, 0x46, 0x1F, 0x10, 0x9E, 0x00, 0x3C, 0x1F, 0x08, 0xB0, 0x04, 0xFF, 0x2F, 0x1F, 0x08, 0xFC, 0x00, 0x92, 0x1F, 0x08, 0x7C, 0x07, 0x90, 0x17, 0x10, 0x04, 0x00, 0x0F, 0x10, 0x03, 0x16, 0x0F, 0x32, 0x05, 0xFF, 0xB5, 0x1F, 0x10, 0x28, 0x00, 0x3D, 0x0F, 0x8C, 0x03, 0x7C, 0x1F, 0x10, 0x28, 0x02, 0xFF, 0x5D, 0x0F, 0xC8, 0x01, 0x3B, 0x1F, 0x08, 0x0A, 0x0B, 0x2B, 0x0F, 0xFC, 0x01, 0xFF, 0xA5, 0x0F, 0xE4, 0x03, 0x10, 0x0E, 0x1E, 0x02, 0x0F, 0x16, 0x0C, 0xFF, 0xFF, 0x1A, 0x1F, 0x08, 0x5C, 0x02, 0x14, 0x0F, 0xB4, 0x0B, 0x05, 0x1F, 0x08, 0x12, 0x00, 0x04, 0x04, 0x90, 0x02, 0x0F, 0x98, 0x02, 0x02, 0x0F, 0x74, 0x00, 0x18, 0x0E, 0x0A, 0x00, 0x1E, 0x08, 0x10, 0x0A, 0x0F, 0x34, 0x0A, 0x1C, 0x0F, 0x34, 0x03, 0x05, 0x1E, 0x08, 0xEE, 0x00, 0x0F, 0x7C, 0x00, 0x08, 0x0E, 0xE8, 0x00, 0x0E, 0x64, 0x07, 0x0F, 0x8A, 0x02, 0xFF, 0x10, 0x0E, 0x20, 0x02, 0x0F, 0xE6, 0x04, 0x0F, 0x0F, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x62, 0x0F, 0x72, 0x05, 0x98, 0x0F, 0x20, 0x06, 0xFF, 0xFF, 0xFF, 0xFF, 0x82, 0x28, 0x08, 0x01, 0xB6, 0x0A, 0x0F, 0xC0, 0x0A, 0xFF, 0xFF, 0xAD, 0x0E, 0x1C, 0x19, 0x0F, 0xF6, 0x07, 0xB3, 0x1F, 0x08, 0x38, 0x00, 0x25, 0x0F, 0x8A, 0x08, 0xBC, 0x0F, 0xD0, 0x00, 0x73, 0x1F, 0x08, 0x08, 0x00, 0x2C, 0x0E, 0xB2, 0x00, 0x0F, 0x0A, 0x00, 0x2E, 0x0F, 0x44, 0x03, 0x3D, 0x0F, 0x40, 0x00, 0x5D, 0x0E, 0xB8, 0x03, 0x0F, 0x98, 0x03, 0x8E, 0x1E, 0x08, 0xB4, 0x00, 0x0F, 0x50, 0x00, 0x33, 0x0F, 0xE0, 0x0D, 0xFF, 0x09, 0x0E, 0xB0, 0x15, 0x0F, 0x5E, 0x09, 0x91, 0x0F, 0xB8, 0x03, 0x08, 0x0F, 0x18, 0x0E, 0xFF, 0xFF, 0x12, 0x1F, 0x10, 0x04, 0x00, 0x05, 0x0F, 0x88, 0x05, 0x49, 0x0F, 0x8C, 0x1F, 0xFF, 0x1D, 0x2E, 0x10, 0x01, 0x30, 0x01, 0x0F, 0x68, 0x00, 0x45, 0x0F, 0x9E, 0x12, 0xFF, 0x54, 0x1F, 0x08, 0x80, 0x1F, 0x27, 0x0F, 0x18, 0x0C, 0x70, 0x0F, 0x44, 0x1D, 0x09, 0x1F, 0x08, 0x64, 0x21, 0xFF, 0xC1, 0x0F, 0xBC, 0x00, 0x21, 0x0F, 0x54, 0x06, 0x02, 0x1F, 0x01, 0x68, 0x06, 0x00, 0x0F, 0xE0, 0x06, 0x54, 0x0F, 0x0C, 0x00, 0x0A, 0x0F, 0x78, 0x01, 0x01, 0x0F, 0xE4, 0x2B, 0x48, 0x0E, 0x30, 0x03, 0x0F, 0x64, 0x27, 0x28, 0x0F, 0x5E, 0x18, 0xF2, 0x0F, 0xEC, 0x2D, 0x43, 0x1F, 0x10, 0xC8, 0x04, 0xFF, 0x6F, 0x0E, 0x5A, 0x06, 0x0F, 0x18, 0x08, 0xFF, 0x02, 0x1F, 0x10, 0x8A, 0x1D, 0xFF, 0x55, 0x0F, 0x14, 0x2F, 0x5D, 0x1F, 0x10, 0xF2, 0x1D, 0xFF, 0x61, 0x4F, 0x01, 0x00, 0x08, 0xF9, 0x8E, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0x33, 0x1E, 0x08, 0xA0, 0x07, 0x0F, 0x4E, 0x04, 0x80, 0x2F, 0x10, 0x01, 0xC0, 0x04, 0x63, 0x0A, 0x70, 0x0D, 0x0F, 0xE0, 0x2C, 0x04, 0x0E, 0x7E, 0x00, 0x0F, 0x6C, 0x39, 0x42, 0x09, 0x1C, 0x00, 0x0F, 0x10, 0x01, 0x07, 0x0F, 0x2A, 0x1B, 0x15, 0x0F, 0x04, 0x06, 0x72, 0x1E, 0x08, 0x82, 0x00, 0x0F, 0x6E, 0x02, 0x4E, 0x0F, 0x62, 0x00, 0x3D, 0x1F, 0x08, 0xE8, 0x3B, 0x70, 0x0F, 0x06, 0x13, 0x0C, 0x0F, 0x50, 0x1E, 0x42, 0x1F, 0x4C, 0x94, 0x17, 0x0A, 0x0F, 0x40, 0x11, 0x00, 0x0F, 0xA4, 0x29, 0xFF, 0x07, 0x1F, 0x08, 0x88, 0x13, 0xB3, 0x0F, 0x88, 0x3B, 0xFF, 0xA5, 0x0E, 0xA0, 0x03, 0x0E, 0x3E, 0x06, 0x0F, 0x28, 0x14, 0x25, 0x2F, 0x08, 0x01, 0x7E, 0x14, 0x32, 0x0F, 0xC6, 0x17, 0xFF, 0x4D, 0x0F, 0x78, 0x22, 0x0A, 0x0F, 0x60, 0x01, 0xFF, 0x8F, 0x0E, 0xA8, 0x1E, 0x0F, 0x12, 0x31, 0xFF, 0x28, 0x1F, 0x08, 0xD0, 0x1E, 0x04, 0x0E, 0x90, 0x19, 0x0E, 0xA0, 0x19, 0x0F, 0x74, 0x22, 0x78, 0x0F, 0x14, 0x05, 0xFF, 0xD7, 0x0F, 0xA4, 0x28, 0x32, 0x0F, 0x38, 0x06, 0xFF, 0x1B, 0x0E, 0xAA, 0x26, 0x0F, 0x50, 0x0B, 0xAF, 0x1F, 0x08, 0xF8, 0x0B, 0xFC, 0x1F, 0x08, 0xB8, 0x01, 0xFF, 0x57, 0x08, 0x30, 0x26, 0x1F, 0x10, 0xA8, 0x28, 0xC4, 0x0F, 0xE0, 0x06, 0x8E, 0x0F, 0x8C, 0x48, 0x0D, 0x0F, 0xD4, 0x08, 0xFF, 0xED, 0x0E, 0xE0, 0x07, 0x0F, 0x24, 0x35, 0xBC, 0x0F, 0x4C, 0x0F, 0xFF, 0x79, 0x0F, 0x38, 0x16, 0x09, 0x1F, 0x10, 0xFC, 0x00, 0xE0, 0x0E, 0xF8, 0x00, 0x0F, 0x70, 0x02, 0x08, 0x0F, 0x1C, 0x07, 0x9D, 0x0F, 0x44, 0x3A, 0xFF, 0xDD, 0x0F, 0x4C, 0x00, 0x0A, 0x0F, 0x8C, 0x54, 0x1D, 0x0F, 0x08, 0x05, 0xE4, 0x1F, 0x4C, 0xE2, 0x23, 0x2B, 0x0F, 0x6C, 0x2E, 0xFF, 0x63, 0x0E, 0xEC, 0x04, 0x0F, 0xF8, 0x19, 0x8A, 0x0F, 0x2A, 0x24, 0xBD, 0x0F, 0x20, 0x1A, 0xFF, 0x3D, 0x0F, 0x8C, 0x4C, 0xFF, 0xFF, 0xFF, 0xFF, 0xBB, 0x08, 0xDC, 0x1B, 0x0F, 0x24, 0x04, 0xFF, 0xFF, 0xFF, 0x68, 0x0F, 0x80, 0x03, 0x21, 0x1F, 0x08, 0xA8, 0x04, 0x3D, 0x0F, 0xFC, 0x08, 0xB8, 0x0F, 0x1A, 0x25, 0x1A, 0x0E, 0x8C, 0x11, 0x0F, 0xB4, 0x08, 0xFF, 0xFF, 0xFF, 0x15, 0x0E, 0x34, 0x03, 0x0F, 0xB4, 0x4F, 0xFF, 0xFF, 0x41, 0x1F, 0x10, 0xE4, 0x0F, 0x69, 0x0F, 0x1E, 0x58, 0xFF, 0x4F, 0x1F, 0x08, 0x32, 0x01, 0xA0, 0x0E, 0x5A, 0x08, 0x0F, 0x94, 0x2D, 0x35, 0x1F, 0x08, 0x94, 0x42, 0x29, 0x0F, 0x48, 0x10, 0xFF, 0xFF, 0x0A, 0x0E, 0xA8, 0x07, 0x0F, 0xEC, 0x56, 0xFF, 0xFF, 0x61, 0x08, 0x50, 0x48, 0x0F, 0xDE, 0x02, 0x12, 0x0F, 0x10, 0x0F, 0x21, 0x0F, 0x84, 0x07, 0x49, 0x0F, 0x94, 0x0D, 0xFF, 0xFF, 0x80, 0x0E, 0xFE, 0x07, 0x0F, 0xCC, 0x62, 0xFF, 0xFF, 0xFF, 0xFF, 0xC1, 0x0F, 0x4A, 0x04, 0x2A, 0x0F, 0xF4, 0x15, 0xFF, 0xFF, 0xE2, 0x0F, 0xE4, 0x55, 0x0E, 0x0F, 0x1C, 0x56, 0x21, 0x0F, 0x3C, 0x02, 0xFF, 0xDD, 0x1F, 0x10, 0xAC, 0x85, 0xE1, 0x0F, 0xC6, 0x86, 0xFF, 0xFF, 0xFF, 0xD1, 0x0E, 0x32, 0x79, 0x0F, 0x10, 0x79, 0x09, 0x0E, 0x28, 0x00, 0x0E, 0x08, 0x00, 0x0E, 0x20, 0x00, 0x0F, 0x32, 0x46, 0x13, 0x0F, 0xB0, 0x64, 0x00, 0x0F, 0x8E, 0x7A, 0x0D, 0x0F, 0xAC, 0x3D, 0xC6, 0x0E, 0xB2, 0x68, 0x0F, 0xD2, 0x4C, 0x39, 0x0E, 0x4E, 0x01, 0x0F, 0x1C, 0x29, 0xFF, 0xFF, 0x1E, 0x0F, 0x32, 0x02, 0xFF, 0xFF, 0xFF, 0xB9, 0x0F, 0x9C, 0x1C, 0xFF, 0x76, 0x0F, 0x18, 0x1E, 0xFF, 0x6F, 0x1F, 0x01, 0x9C, 0x54, 0x10, 0x0F, 0xE4, 0x34, 0xFF, 0xFF, 0x72, 0x2F, 0x08, 0x00, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x34};
    int addr_start;
    int addr_end;

    addr_start = 528;
    addr_end = 262944;


    int lenght = sizeof(data);


    addr_start = MBED_CONF_LORAWAN_UPDATE_CLIENT_SLOT0_FW_ADDRESS;
    
    tr_debug("Programming %d bytes from : %x \n",lenght,(addr_start));
    //int ret_value =  bd->program(data, (addr_start), lenght);
    int ret_value =  block_device.program(data, (addr_start), lenght);

    tr_info("Programming of %d DONE, code %d\n",lenght,ret_value); 
}



