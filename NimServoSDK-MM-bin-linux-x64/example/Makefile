CC  = gcc

ALL: test_pp test_vm test_pv test_hm test_ip test_csp test_csv test_pt
test_pt: test_pt.o
	$(CC) -o ../bin/$@ $^ -L../bin -lNimServoSDK -lGlobal -lAbstractMaster -lDataBase -lsqlite3

test_pt.o:test_pt.c
	$(CC) -c $^ -o $@ -I../inc 

test_pp: test_pp.o
	$(CC) -o ../bin/$@ $^ -L../bin -lNimServoSDK -lGlobal -lAbstractMaster -lDataBase -lsqlite3

test_pp.o:test_pp.c
	$(CC) -c $^ -o $@ -I../inc 
	
test_vm: test_vm.o
	$(CC) -o ../bin/$@ $^ -L../bin -lNimServoSDK -lGlobal -lAbstractMaster -lDataBase -lsqlite3

test_vm.o:test_vm.c
	$(CC) -c $^ -o $@ -I../inc 
	
test_pv: test_pv.o
	$(CC) -o ../bin/$@ $^ -L../bin -lNimServoSDK -lGlobal -lAbstractMaster -lDataBase -lsqlite3

test_pv.o:test_pv.c
	$(CC) -c $^ -o $@ -I../inc 
	
test_hm: test_hm.o
	$(CC) -o ../bin/$@ $^ -L../bin -lNimServoSDK -lGlobal -lAbstractMaster -lDataBase -lsqlite3

test_hm.o:test_hm.c
	$(CC) -c $^ -o $@ -I../inc 
	
test_ip: test_ip.o
	$(CC) -o ../bin/$@ $^ -L../bin -lNimServoSDK -lGlobal -lAbstractMaster -lDataBase -lsqlite3 -lm

test_ip.o:test_ip.c
	$(CC) -c $^ -o $@ -I../inc 
	
test_csp: test_csp.o
	$(CC) -o ../bin/$@ $^ -L../bin -lNimServoSDK -lGlobal -lAbstractMaster -lDataBase -lsqlite3 -lm

test_csp.o:test_csp.c
	$(CC) -c $^ -o $@ -I../inc 
	
test_csv: test_csv.o
	$(CC) -o ../bin/$@ $^ -L../bin -lNimServoSDK -lGlobal -lAbstractMaster -lDataBase -lsqlite3 -lm

test_csv.o:test_csv.c
	$(CC) -c $^ -o $@ -I../inc 

clean:
	rm -f *.o

