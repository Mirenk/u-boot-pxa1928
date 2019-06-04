#include <common.h>
#include <mmc.h>
#include <environment.h>

#define KPP_LEN 10

static int do_kpp(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char  *s;
	char buff[KPP_LEN + 1];
	unsigned char *pvalue;
	int size,rc;

	if (argc == 1) {
		goto print_kpp;
	}

	if (argc != 2) {
		rc = -1;
		goto error;
	}

	memset(buff,0x00,sizeof(buff));

	pvalue = (unsigned char *)(argv [1]);

	size = strlen((char *)pvalue);
	if (size > KPP_LEN)
		size = KPP_LEN;   /* max */

	strncpy(buff,(char *)pvalue,size);

	/* Set the value */
	if(setenv("kpp",buff)) {
		rc = -3;
		goto error;
	}
	saveenv();/* save eMMC or SD */

print_kpp:
	if ((s = getenv("kpp")) != NULL) {
		printf("\nkpp: %s\n\n", s);
	}
	else
	{
		printf("\nkpp: not set\n\n");
	}
	return 0;

error:
	printf("Wrong value for kpp. Error=%d\n\n", rc);
	return rc;
}

U_BOOT_CMD(
	kpp,	2,	0,	do_kpp,
	"Write / Read kpp",
	"[testkeycode]");
