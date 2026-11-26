#include "test.h"


// Main Cpp event loop to run application
void test_cpp()
{
 
}
 
// Define all C function calls from main.c below
extern "C"
{
    void test_c()
    {
        test_cpp();
    }
}