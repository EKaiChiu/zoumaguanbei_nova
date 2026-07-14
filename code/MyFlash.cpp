#include "MyFlash.hpp"

#include "zf_common_headfile.hpp"

static const char *FLASH_PARAM_FILE = "nova_params.txt";
static zf_driver_file_string flash_file(FLASH_PARAM_FILE, "a+");
static bool flash_ready = false;

void MyFlash_Init(void)
{
    flash_file.set_path(FLASH_PARAM_FILE, "a+");
    flash_file.rewind_file();
    flash_ready = true;

    MyFlash_LoadParameters();
    printf("[FLASH] framework ready: %s\r\n", FLASH_PARAM_FILE);
}

void MyFlash_LoadParameters(void)
{
    if (!flash_ready)
    {
        flash_file.set_path(FLASH_PARAM_FILE, "a+");
        flash_ready = true;
    }

    flash_file.rewind_file();

    // Framework only: parameter parsing will be added here later.
}

void MyFlash_SaveParameters(void)
{
    if (!flash_ready)
    {
        flash_file.set_path(FLASH_PARAM_FILE, "a+");
        flash_ready = true;
    }

    flash_file.set_path(FLASH_PARAM_FILE, "w+");
    flash_file.rewind_file();

    // Framework only: parameter writing will be added here later.
    printf("[FLASH] framework save complete\r\n");
}
