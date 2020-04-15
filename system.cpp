void System::matrix_mul_time_optimized(int A_row, int A_col, int B_row, int B_col) 
{
    AddrT storage_start_address =  (AddrT)_ncols *_nrows * _nblocks * _ntiles / 4 * 3; // use the last 3/4 for storage units
    AddrT pim_start_address = 0;

    //The data will be arranged in block in this way
    //[0]-----[1]-----[2:11]-----[12:21]-----[22:31]
    //MUL1---MUL2-------A-----------B----------PS---

    int mul1_p = 0;
    int mul2_p = 1;
    int a_width = 1;     // Number of row in block which are occupied by A
    int b_width = 1;     // Number of row in block which are occupied by B
    int height = 40;     // Number of row in block which are occupied by B
    int a_p    = 0;
    int b_p    = mul1_p;
    int ps_p   = b_p + 1;
    int block_row = 32; // Number of data can be stored in one row of block
    int block_col = 1024; // Number of data can be stored in one col of block
    int no_block = A_col/a_width * B_col/b_width * (A_col/height);
    int no_block_same_a = B_col/b_width* (A_col/height);
    int no_block_same_b = A_row/a_width;
    int a_sb_y = ceil((float)A_row/(float)a_width );
    int b_sb_x = ceil((float)B_col/(float) b_width);
    int last_sum_p = height;


    AddrT data_a_p = 0;
    AddrT data_b_p = 0;
    AddrT pim_a_p = 0;
    AddrT pim_b_p = 0;
    AddrT sum_p = 4000*_ncols*_nrows; //TODO


    data_a_p = storage_start_address;
    pim_a_p  = pim_start_address;
    data_b_p = storage_start_address + (AddrT) 2*256*1024*1024*32;
    pim_b_p  = pim_start_address;


   	std::vector<Request> requests;
   	Request *request;

   	//1600
    //Fill all A to PIM unit
    request = new Request(Request::Type::SystemCol2Col);
    for (int n_a_row = 0; n_a_row < A_row * (A_row/height); n_a_row++){//i: index of current A row
    	for (int n_blk = 0; n_blk < no_block_same_a; n_blk++){// no of blocks used
    		for (int ii = 0; ii < A_row/20; ii++){// no of blocks used
    			request->addAddr(data_a_p + (AddrT)(n_a_row *2 + ii), 20*32);
    			request->addAddr(pim_a_p  + (AddrT) (n_a_row * no_block_same_a*_ncols*_nrows + n_blk *_ncols*_nrows + a_p + ii*2) ,20*32);
    			//request->addAddr(data_b_p + (AddrT)(n_blk *2) + ii, 20*32);
    			//request->addAddr(pim_b_p  + (AddrT) (n_a_row * no_block_same_a*_ncols*_nrows + n_blk *_ncols*_nrows + b_p + ii),20*32);
    		}
    	}
    }

    //Fill block-col of B to each PIM

    for (int n_b_col = 0; n_b_col < B_col * B_col/height; n_b_col++){//i: index of current A row
    	for (int n_blk = 0; n_blk < no_block_same_b; n_blk++){// no of blocks used
    		for (int ii = 0; ii < B_col/20; ii++){// no of blocks used
    			request->addAddr(data_b_p + (AddrT)(n_b_col *2) + ii, 20*32);
    			request->addAddr(pim_b_p  + (AddrT) (n_blk * no_block_same_a * _ncols*_nrows + (n_b_col/b_width)) *_ncols*_nrows + b_p + n_b_col%b_width + ii*2 ,20*32);
    		}
    	}
    }
    requests.push_back(*request);

    for (unsigned int i = 0; i < requests.size(); i++){
        //sendRequest(requests[i]);
    }
   	std::vector<Request>().swap(requests);

//shift
    request = new Request(Request::Type::ColBitwise);
    for (int n_b_blk= 0; n_b_blk < no_block;n_b_blk++){//i: index of current A row
    	request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + a_p + 2) ,20*32);
    	request->addAddr(pim_b_p  + (AddrT) (n_b_blk * _ncols*_nrows + b_p + 3) ,20*32);
    }
    requests.push_back(*request);

    for (unsigned int i = 0; i < requests.size(); i++){
      sendRequest(requests[i]);
    }
   	std::vector<Request>().swap(requests);

//ColMv
    request = new Request(Request::Type::ColMv);
    for (int n_b_blk= 0; n_b_blk < no_block;n_b_blk++){//i: index of current A row
    	request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + a_p + 2) ,20*32);
    	request->addAddr(pim_b_p  + (AddrT) (n_b_blk * _ncols*_nrows + b_p + 3) ,20*32);
    }
    requests.push_back(*request);

    for (unsigned int i = 0; i < requests.size(); i++){
      //  sendRequest(requests[i]);
    }
   	std::vector<Request>().swap(requests);

//multiplication
    request = new Request(Request::Type::RowMul);
    for (int n_b_blk= 0; n_b_blk < no_block;n_b_blk++){//i: index of current A row
    	for (int no_h = 0; no_h < height;no_h++){//i: index of current A row
    		request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + no_h*_ncols) ,2*32); //The results is stored at sum_p
    	}
    }
    requests.push_back(*request);

    for (unsigned int i = 0; i < requests.size(); i++){
       // sendRequest(requests[i]);
    }
   	std::vector<Request>().swap(requests);


//Addition
    for (int no_h = 1; no_h <height;no_h++){
    	request = new Request(Request::Type::ColAdd);
    	for (int n_b_blk= 0; n_b_blk < no_block;n_b_blk++){//i: index of current A row
    		request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + no_h*2*_ncols) ,2*32);//The results will be stored at the end of this col
    	}
    	requests.push_back(*request);
    	for (unsigned int i = 0; i < requests.size(); i++){
    		//sendRequest(requests[i]);
    	}
    	std::vector<Request>().swap(requests);
    }



//Send back
    request = new Request(Request::Type::SystemCol2Col);
   // for (int n_b_blk= 0; n_b_blk < no_block;n_b_blk++){
    for (int n_b_blk= 389; n_b_blk < 400;n_b_blk++){
    	request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + 2*height -1 ) ,32);//The results will be stored at the end of this col
    	request->addAddr(data_a_p  + (AddrT) (n_b_blk * _ncols*_nrows ) ,32);//The results will be stored at the end of this col
    }
    requests.push_back(*request);
    for (unsigned int i = 0; i < requests.size(); i++){
    	//sendRequest(requests[i]);
    }
    std::vector<Request>().swap(requests);


//To see the final cycle
    request = new Request(Request::Type::ColMv);
    request->addAddr(pim_a_p  + (AddrT) ( a_p + 2) ,20*32);
    request->addAddr(pim_b_p  + (AddrT) ( b_p + 3) ,20*32);
    requests.push_back(*request);
    sendRequest(request[0]);

}
