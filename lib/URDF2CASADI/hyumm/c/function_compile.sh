gcc -fPIC -shared -O3 hyumm_C.c -o hyumm_C.so -lm
gcc -fPIC -shared -O3 hyumm_G.c -o hyumm_G.so -lm
gcc -fPIC -shared -O3 hyumm_M.c -o hyumm_M.so -lm
gcc -fPIC -shared -O3 hyumm_Minv.c -o hyumm_Minv.so -lm
gcc -fPIC -shared -O3 hyumm_fd.c -o hyumm_fd.so -lm
gcc -fPIC -shared -O3 hyumm_fk.c -o hyumm_fk.so -lm
gcc -fPIC -shared -O3 hyumm_fk_ee.c -o hyumm_fk_ee.so -lm
gcc -fPIC -shared -O3 hyumm_fkrot_ee.c -o hyumm_fkrot_ee.so -lm
gcc -fPIC -shared -O3 hyumm_id.c -o hyumm_id.so -lm
gcc -fPIC -shared -O3 hyumm_J_b.c -o hyumm_J_b.so -lm
gcc -fPIC -shared -O3 hyumm_J_fd.c -o hyumm_J_fd.so -lm
gcc -fPIC -shared -O3 hyumm_J_id.c -o hyumm_J_id.so -lm
gcc -fPIC -shared -O3 hyumm_J_s.c -o hyumm_J_s.so -lm
gcc -fPIC -shared -O3 hyumm_CoM_x.c -o hyumm_CoM_x.so -lm

mv ./*.so ../

