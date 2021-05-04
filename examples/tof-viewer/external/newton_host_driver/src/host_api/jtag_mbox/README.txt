There is no makefile For the .c files:
read_state_TEST.c
read_state_BLANK.c
reset_OTHERS.c
reset_TEST.c
trans_TEST_to_PROD.c
trans_TEST_to_RETEST.c
trans_BLANK_to_TEST.c

Follow the following example if you want to compile an executable for the .c file.

cp read_state_TEST.c main.c
make
cp example read_state_TEST


