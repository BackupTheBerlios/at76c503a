/* This file includes the old style firmwares and outputs new, binary files. */

#include <stdio.h>
#include <string.h>
#include <byteswap.h>
#include <endian.h>

typedef unsigned char u8;
typedef unsigned char u_int8_t;
typedef unsigned char UCHAR;
typedef unsigned short USHORT;
typedef unsigned int u32;

#define BOARDTYPE_503_INTERSIL_3861 1
#define BOARDTYPE_503_INTERSIL_3863 2
#define BOARDTYPE_503_RFMD          3
#define BOARDTYPE_503_RFMD_ACC      4
#define BOARDTYPE_505_RFMD          5
#define BOARDTYPE_505_RFMD_2958     6
#define BOARDTYPE_505A_RFMD_2958    7
#define BOARDTYPE_505AMX_RFMD       8

/* The struct of the firmware header: */
struct at76c50x_fw_header {
        u32 crc;             // CRC32 of the whole image (seed ~0, no post-process)
        u32 board_id;        // BOARDTYPE_xxx
        u32 version;         // firmware version code
        u32 str_offset;      // printable string offset (copyright)
        u32 internal_offset; // internal firmware image offset
        u32 internal_len;    // internal firmware image length
        u32 external_offset; // external firmware image offset
        u32 external_len;    // external firmware image length
};

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define cpu_to_le32(x) (x)
#elif __BYTE_ORDER == __BIG_ENDIAN
#define cpu_to_le32(x) bswap_32(x)
#else
#error "Unknown endianess"
#endif

// round to next multiple of four
#define QUAD(x) ((x) % 4 ? (x) + (4 - ((x)%4)) : (x))


#include "atmel_intersil_fw.h"
#include "atmel_at76c503_i3863_fw.h"
#include "atmel_at76c503_rfmd2_fw.h"
#include "atmel_at76c503_rfmd_acc_fw.h"
#include "atmel_at76c505_rfmd.h"
#include "atmel_rfmd2958_fw.h"
#include "atmel_rfmd2958-smc_fw.h"
#include "atmel_rfmd_fw.h"

#define BuildInInternalFW fw_at76c503_int
#define BuildInExternalFW fw_at76c503_ext
#define BuildInInternalFWLen fw_at76c503_il
#define BuildInExternalFWLen fw_at76c503_el
#include "fw-at76c503.h"
#undef BuildInInternalFW
#undef BuildInExternalFW
#undef BuildInInternalFWLen
#undef BuildInExternalFWLen

#define BuildInInternalFW fw_at76c505_int
#define BuildInExternalFW fw_at76c505_ext
#define BuildInInternalFWLen fw_at76c505_il
#define BuildInExternalFWLen fw_at76c505_el
#include "fw-at76c505.h"
#undef BuildInInternalFW
#undef BuildInExternalFW
#undef BuildInInternalFWLen
#undef BuildInExternalFWLen

#define BuildInInternalFW fw_at76c505_2958_int
#define BuildInExternalFW fw_at76c505_2958_ext
#define BuildInInternalFWLen fw_at76c505_2958_il
#define BuildInExternalFWLen fw_at76c505_2958_el
#include "fw-at76c505-2958.h"
#undef BuildInInternalFW
#undef BuildInExternalFW
#undef BuildInInternalFWLen
#undef BuildInExternalFWLen

#define BuildInInternalFW fw_at76c505a_2958_int
#define BuildInExternalFW fw_at76c505a_2958_ext
#define BuildInInternalFWLen fw_at76c505a_2958_il
#define BuildInExternalFWLen fw_at76c505a_2958_el
#include "fw-at76c505a-2958.h"
#undef BuildInInternalFW
#undef BuildInExternalFW
#undef BuildInInternalFWLen
#undef BuildInExternalFWLen

#define BuildInInternalFW fw_at76c505amx_int
#define BuildInExternalFW fw_at76c505amx_ext
#define BuildInInternalFWLen fw_at76c505amx_il
#define BuildInExternalFWLen fw_at76c505amx_el
#include "fw-at76c505amx.h"
#undef BuildInInternalFW
#undef BuildInExternalFW
#undef BuildInInternalFWLen
#undef BuildInExternalFWLen


static struct fw {
	const char *filename;
	u32 board_id;
	u8 major, minor, patch, build;
	const char *str_id;
	u8 *intfw;
	u32 intfw_sz;
	u8 *extfw;
	u32 extfw_sz;
} fws[] = {
	{ "at76c503-rfmd", BOARDTYPE_503_RFMD,
	  0, 90, 2, 140, "503 RFMD "
	  "Copyright (c) 2004 by Atmel Corporation",
	  atmel_at76c503_rfmd2_fw_int, sizeof(atmel_at76c503_rfmd2_fw_int),
	  atmel_at76c503_rfmd2_fw_ext, sizeof(atmel_at76c503_rfmd2_fw_ext)},

	{ "at76c503-rfmd", BOARDTYPE_503_RFMD,
	  1, 101, 0, 84, "503 RFMD "
	  "Copyright (c) 2004 by Atmel Corporation",
	  atmel_fw_rfmd_int, sizeof(atmel_fw_rfmd_int),
	  atmel_fw_rfmd_ext, sizeof(atmel_fw_rfmd_ext)},

	{ "at76c503-rfmd", BOARDTYPE_503_RFMD,
	  1, 103, 0, 175, "503 RFMD "
	  "Copyright (c) 2004 by Atmel Corporation",
	  fw_at76c503_int, sizeof(fw_at76c503_int),
	  fw_at76c503_ext, sizeof(fw_at76c503_ext)},

	{ "at76c503-rfmd-acc", BOARDTYPE_503_RFMD_ACC,
	  1, 101, 0, 84, "503 RFMD Accton design "
	  "Copyright (c) 2004 by Atmel Corporation",
	  atmel_at76c503_rfmd_acc_fw_int, sizeof(atmel_at76c503_rfmd_acc_fw_int),
	  atmel_at76c503_rfmd_acc_fw_ext, sizeof(atmel_at76c503_rfmd_acc_fw_ext)},

	{ "at76c503-i3861", BOARDTYPE_503_INTERSIL_3861,
	  0, 90, 0, 44, "Intersil 3861 "
	  "Copyright (c) 2004 by Atmel Corporation",
	  atmel_fw_intersil_int, sizeof(atmel_fw_intersil_int),
	  atmel_fw_intersil_ext, sizeof(atmel_fw_intersil_ext)},

	{ "at76c503-i3863", BOARDTYPE_503_INTERSIL_3863,
	  0, 90, 0, 44, "Intersil 3863 "
	  "Copyright (c) 2004 by Atmel Corporation",
	  atmel_at76c503_i3863_fw_int, sizeof(atmel_at76c503_i3863_fw_int),
	  atmel_at76c503_i3863_fw_ext, sizeof(atmel_at76c503_i3863_fw_ext)},

	{ "at76c505-rfmd", BOARDTYPE_505_RFMD,
	  0, 91, 0, 4, "505 RFMD "
	  "Copyright (c) 2004 by Atmel Corporation",
	  atmel_at76c505_rfmd_fw_int, sizeof(atmel_at76c505_rfmd_fw_int),
	  atmel_at76c505_rfmd_fw_ext, sizeof(atmel_at76c505_rfmd_fw_ext)},

	{ "at76c505-rfmd", BOARDTYPE_505_RFMD,
	  1, 103, 0, 175, "505 RFMD "
	  "Copyright (c) 2004 by Atmel Corporation",
	  fw_at76c505_int, sizeof(fw_at76c505_int),
	  fw_at76c505_ext, sizeof(fw_at76c505_ext)},

	{ "at76c505-rfmd2958", BOARDTYPE_505_RFMD_2958,
	  1, 101, 0, 86, "505 RFMD2958 "
	  "Copyright (c) 2004 by Atmel Corporation",
	  atmel_fw_rfmd2958_int, sizeof(atmel_fw_rfmd2958_int),
	  atmel_fw_rfmd2958_ext, sizeof(atmel_fw_rfmd2958_ext)},

	{ "at76c505-rfmd2958", BOARDTYPE_505_RFMD_2958,
	  1, 103, 0, 175, "505 RFMD2958 "
	  "Copyright (c) 2004 by Atmel Corporation",
	  fw_at76c505_2958_int, sizeof(fw_at76c505_2958_int),
	  fw_at76c505_2958_ext, sizeof(fw_at76c505_2958_ext)},

	{ "at76c505a-rfmd2958", BOARDTYPE_505A_RFMD_2958,
	  1, 102, 0, 113, "505A RFMD 2958 "
	  "Copyright (c) 2004 by Atmel Corporation",
	  atmel_fw_rfmd2958_smc_int, sizeof(atmel_fw_rfmd2958_smc_int),
	  atmel_fw_rfmd2958_smc_ext, sizeof(atmel_fw_rfmd2958_smc_ext)},

	{ "at76c505a-rfmd2958", BOARDTYPE_505A_RFMD_2958,
	  1, 103, 0, 175, "505A RFMD 2958 "
	  "Copyright (c) 2004 by Atmel Corporation",
	  fw_at76c505a_2958_int, sizeof(fw_at76c505a_2958_int),
	  fw_at76c505a_2958_ext, sizeof(fw_at76c505a_2958_ext)},

	{ "at76c505amx", BOARDTYPE_505AMX_RFMD,
	  1, 103, 0, 175, "505AMX RFMD "
	  "Copyright (c) 2004 by Atmel Corporation",
	  fw_at76c505amx_int, sizeof(fw_at76c505amx_int),
	  fw_at76c505amx_ext, sizeof(fw_at76c505amx_ext)},
};

static int nr_fws = sizeof(fws) / sizeof(struct fw);
static const u8 zeros[] = {0,0,0};

#define _CRCPOLY_LE 0xedb88320
static u32 crc32 (u32 crc, u8 const *p, size_t len)
{
        int i;
        while (len--) {
                crc ^= *p++;
                for (i = 0; i < 8; i++)
                        crc = (crc >> 1) ^ ((crc & 1) ? _CRCPOLY_LE : 0);
        }
        return crc;
}

int main(void)
{
	int i;
	FILE *f;
	struct fw *fw;
	struct at76c50x_fw_header hd;
	u32 internal_offset, external_offset, str_offset, crc;
	char ver_id[16];
	char str_id[128];
	char filename[128];

	for(i=0,fw=fws; i < nr_fws; fw++,i++) {
		memset(ver_id, 0, sizeof(ver_id));
		snprintf(ver_id, sizeof(ver_id), "%d.%d.%d-%d",
			 fw->major, fw->minor, fw->patch, fw->build);
		memset(str_id, 0, sizeof(str_id));
		snprintf(str_id, sizeof(str_id), "%s %s", ver_id, fw->str_id);
		memset(filename, 0, sizeof(filename));
		snprintf(filename, sizeof(filename), "atmel_%s-%s.bin",
			 fw->filename, ver_id);

		if ((f=fopen(filename, "w")) == NULL) {
			fprintf(stderr,"#ERR cannot open %s for writing (errno %m)\n",
				filename);
			continue;
		}

		crc = ~0; /* the initial seed */
		hd.board_id = cpu_to_le32(fw->board_id);
		hd.version = cpu_to_le32((((((fw->major << 8) +
			fw->minor) << 8) + fw->patch) << 8) + fw->build);
		// string area starts after header
		str_offset = sizeof(struct at76c50x_fw_header);
		hd.str_offset = cpu_to_le32(str_offset);
		internal_offset = str_offset + strlen(str_id) + 1;
		internal_offset = QUAD(internal_offset);
		hd.internal_offset = cpu_to_le32(internal_offset);
		hd.internal_len = cpu_to_le32(fw->intfw_sz);
		external_offset = internal_offset + fw->intfw_sz;
		external_offset = QUAD(external_offset);
		hd.external_offset = cpu_to_le32(external_offset);
		hd.external_len = cpu_to_le32(fw->extfw_sz);

		/* calc crc */
		/* the header */
		crc = crc32(crc, (u8 *)&hd.board_id, 0x20-0x4);
		/* the string */
		crc = crc32(crc, (u8 *)str_id, strlen(str_id) +1);
		/* zeros in gap */
		crc = crc32(crc, zeros, internal_offset - 
			       (str_offset + strlen(str_id)  + 1));
		/* internal fw */
		crc = crc32(crc, fw->intfw, fw->intfw_sz);
		/* zeros in gap */
		crc = crc32(crc, zeros, external_offset - 
			       (internal_offset + fw->intfw_sz));
		/* external fw */
		crc = crc32(crc, fw->extfw, fw->extfw_sz);
		hd.crc = cpu_to_le32(crc);

#define FWRITE(ptr,len,fp) \
  if ((len) > 0) {\
    if (fwrite(ptr,len,1,fp) < 1) {\
      fprintf(stderr,"#ERR failed to write %d bytes, errno %m\n", (int)(len));\
      fclose(fp);\
      continue;\
    }\
  }

		FWRITE((u8 *)&hd, sizeof(hd),f);
		FWRITE(str_id, strlen(str_id)+1, f);
		FWRITE(zeros, internal_offset - 
		       (str_offset + strlen(str_id)  + 1), f);
		FWRITE(fw->intfw, fw->intfw_sz, f);
		FWRITE(zeros, external_offset - 
		       (internal_offset + fw->intfw_sz), f);
		FWRITE(fw->extfw, fw->extfw_sz, f);

		fclose(f);
	}

	return 0;
}

