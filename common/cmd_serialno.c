#include <common.h>
#include <mmc.h>
#include <environment.h>

#define SERIALNUM_MAX_LEN			15
#define SERIAL_NO_OFF				0x00
#define SERIAL_NO_CHECK_OFF			0x11

static int do_serialno(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned char *pvalue;
	unsigned char  tmp[SERIALNUM_MAX_LEN+1];
	int rc, size, i,check;
	unsigned char buff[512];
	
	memset(buff,0x00,sizeof(buff));
	memset(tmp,0x00,sizeof(tmp));

	if (argc == 1) {
		goto print_serial_no;
	}

	if (argc != 2) {
		rc = -1;
		goto error;
	}
	pvalue = (unsigned char *)(argv [1]);
	size = strlen((char *)pvalue);
	if (size > SERIALNUM_MAX_LEN)
		size = SERIALNUM_MAX_LEN;   /* max */

	for ( i = 0,check = 0; i < size; i++, pvalue++ ) {
		tmp[i] = *pvalue;
		check += *pvalue;
	}
	tmp[i] = 0x00;
	
	if(read_serialmac(buff)) {
		rc = -2;
		goto error;
	}
	
	memcpy(buff,tmp,sizeof(tmp));
	memcpy(buff+SERIAL_NO_CHECK_OFF,&check,sizeof(check));

	/* Set the value */
	if(save_serialmac(buff)) {
		rc = -3;
		goto error;
	}
	
print_serial_no:
	if(read_serialmac(buff)) {
		rc = -4;
		goto error;
	}
	printf("\nSerial Number: %s\n\n", buff+SERIAL_NO_OFF);
	
	return 0;

error:
	printf("Wrong value for serialno. Error=%d\n\n", rc);
	return rc;
}

U_BOOT_CMD(
	serialno,	2,	0,	do_serialno,
	"Write / Read the serial number",
	"[serial number chars]");
