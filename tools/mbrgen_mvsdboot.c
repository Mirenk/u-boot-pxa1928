#define CONFIG_USE_CUSTOM_PARAMS
#define CONFIG_IGNORE_END_EXCEED_NEXT_START

#define DEFAULT_MBR_NAME1	"primary_gpt"
#define DEFAULT_MBR_NAME2	"second_gpt"
#define SECTOR_SIZE		512
#define GPT_HEADER_SIZE		512
#define MAX_PART_ENTRY		28
#define MAX_NAME_SZ		36
#define GPT_SIZE		128
#define ENTRY_NUM		28
#define FIRST_USABLE_LBA	8
#define HEADER_SIZE		92

#include "mbrgen.c"
