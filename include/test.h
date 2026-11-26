#ifndef TEST_H
#define TEST_H

void test_cpp();

#ifdef __cplusplus
extern "C" {
#endif

void test_c();

#ifdef __cplusplus
}
#endif


#endif // TEST_H