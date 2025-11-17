#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* XiUOS includes */
#ifdef CONFIG_ADD_XIZI_FEATURES
#include <xizi.h>
#endif

#ifdef CONFIG_ADD_NUTTX_FEATURES
#include <nuttx/config.h>
#endif

/* PikaPython includes */
#include "pika_port.h"

/* External PikaPython application functions */
extern int pika_app_init(void);
extern int pika_app_run(void);
extern int pika_app_deinit(void);
extern int pika_app_is_initialized(void);
extern int pika_app_load_script(const char* filename);

/* Test function for PikaPython */
int test_pikapython(void)
{
    int ret;
    
    printf("\n=== PikaPython Test for XiUOS ===\n");
    
    /* Initialize PikaPython runtime */
    printf("Initializing PikaPython...\n");
    ret = pika_app_init();
    if (ret != 0) {
        printf("Failed to initialize PikaPython: %d\n", ret);
        return -1;
    }
    
    /* Check if runtime is initialized */
    if (!pika_app_is_initialized()) {
        printf("PikaPython runtime not initialized\n");
        return -1;
    }
    
    printf("PikaPython runtime initialized successfully\n");
    
    /* Run embedded Python script */
    printf("Running embedded Python script...\n");
    ret = pika_app_run();
    if (ret != 0) {
        printf("Failed to run Python script: %d\n", ret);
        pika_app_deinit();
        return -1;
    }
    
    /* Test loading script from file (if file system is enabled) */
#ifdef CONFIG_PIKAPYTHON_ENABLE_FILE_SYSTEM
    printf("Testing file system script loading...\n");
    ret = pika_app_load_script("/app/main.py");
    if (ret == 0) {
        printf("Script loaded from file successfully\n");
        ret = pika_app_run();
        if (ret != 0) {
            printf("Failed to run loaded script: %d\n", ret);
        }
    } else {
        printf("File system script loading not available or failed\n");
    }
#endif
    
    /* Deinitialize PikaPython runtime */
    printf("Deinitializing PikaPython...\n");
    ret = pika_app_deinit();
    if (ret != 0) {
        printf("Failed to deinitialize PikaPython: %d\n", ret);
        return -1;
    }
    
    printf("PikaPython test completed successfully!\n");
    printf("=== End of PikaPython Test ===\n\n");
    
    return 0;
}

/* Main function for standalone test */
#ifdef CONFIG_PIKAPYTHON_STANDALONE_TEST
int main(int argc, char* argv[])
{
    printf("PikaPython Standalone Test\n");
    return test_pikapython();
}
#endif
