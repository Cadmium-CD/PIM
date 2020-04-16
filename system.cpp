#include "backend/System.h"

#include "backend/MemoryBlock.h"
using namespace pimsim;
using namespace std;

System::System(Config* config) 
    : _config(config) 
{
    _nchips = _config->get_nchips();
    _ntiles = _config->get_ntiles();
    _nblocks = _config->get_nblocks();
    _nrows = config->get_nrows();
    _ncols = config->get_ncols();
    _clock_rate = config->get_clock_rate();
    _blockctrl = config->get_blockctrl();
    _tilectrl = config->get_tilectrl();
    _chipctrl = config->get_chipctrl();
    _force_sync = config->getSync();
    if (!(_blockctrl || _tilectrl || _chipctrl))
        _blockctrl = true;
    _blocksize = _nrows * _ncols; // set the banksize based on columns and rows
    rstFile = fopen(config->get_rstfile().c_str(), "w");

    _values = new MemoryCharacteristics();
    for (int i = 0; i < _nchips; i++) {
        MemoryChip* chip = new MemoryChip(_ntiles, _nblocks, _nrows, _ncols, _clock_rate);
        Controller* ctrl = new Controller(chip);
        chip->setId(i);
        chip->setController(ctrl, _clock_rate);
        chip->setParent(NULL);
        chip->setValues(_values);
        _chips.push_back(chip);
    }
    /* Network connection */
    GlobalConnection::Type nt;
    if (_config->get_netscheme() == "mesh") {
        nt = GlobalConnection::Type::Mesh;
    } else if (_config->get_netscheme() == "dragonfly") {
        nt = GlobalConnection::Type::Dragonfly;
    } else {
        nt = GlobalConnection::Type::Ideal;
    }
    _conn = new GlobalConnection(this, nt); 
}

System::~System() 
{
    fclose(rstFile);
}

void
System::addChip(MemoryCharacteristics* values, int n_tiles, int n_blocks, int n_rows, int n_cols, int clock_rate) {
    int global_chip_id = _chips.size();
    MemoryChip* chip = new MemoryChip(n_tiles, n_blocks, n_rows, n_cols, clock_rate);
    Controller* ctrl = new Controller(chip);
    chip->setId(global_chip_id);
    chip->setController(ctrl, clock_rate);
    chip->setParent(NULL);
    chip->setValues(values);
    _chips.push_back(chip);
}

AddrT
System::getAddress(int chip, int tile, int block, int row, int col)
{
    AddrT addr = chip;
    addr *= _ntiles;
    addr += tile;
    addr *= _nblocks;
    addr += block;
    addr *= _nrows;
    addr += row;
    addr *= _ncols;
    addr += col;
    return addr;
}

void
System::getLocation(AddrT addr, 
                    int &chip_idx, 
                    int &tile_idx, 
                    int &block_idx,
                    int &row_idx,
                    int &col_idx)
{
    /* Here is the code for memory mapping 
     * */
    col_idx = addr % _ncols;
    addr /= _ncols;
    row_idx = addr % _nrows;
    addr /= _nrows;
    block_idx = addr % _nblocks;
    addr /= _nblocks;
    tile_idx = addr % _ntiles; 
    addr /= _ntiles;
    chip_idx = addr % _nchips;
}
    
void
System::getLocation(AddrT addr, 
                    int &chip_idx, 
                    int &tile_idx, 
                    int &block_idx) 
{
    /* Here is the code for memory mapping 
     * */
    addr /= _blocksize;
    tile_idx = addr % _nblocks;
    addr /= _nblocks;
    block_idx = addr % _ntiles; 
    addr /= _ntiles;
    chip_idx = addr % _nchips;
}

int
System::sendMoReq(Request& req) 
{
    int tot_clks = 0;
    int chip_idx = 0, tile_idx = 0, block_idx = 0, row_idx = 0, col_idx = 0;
    AddrT addr = req.addr_list[0];
    getLocation(addr, chip_idx, tile_idx, block_idx, row_idx, col_idx);
    req.setLocation(chip_idx, tile_idx, block_idx, row_idx, col_idx);

    tot_clks++;
    bool res = _chips[chip_idx]->receiveReq(req);
    while (!res) {
        tot_clks++;
        _chips[chip_idx]->tick();
        res = _chips[chip_idx]->receiveReq(req);
    }
    return tot_clks;
}

int
System::sendNetReq(Request& req)
{
    int cp1, tl1, bk1, r1, c1,
        cp2, tl2, bk2, r2, c2;
    getLocation(req.addr_list[0], cp1, tl1, bk1, r1, c1);
    getLocation(req.addr_list[1], cp2, tl2, bk2, r2, c2);
    req.setLocation(cp1, tl1, bk1, r1, c1);

    int net_overhead = _conn->getLatency(cp1, cp2, req.size_list[0]);

    TimeT sync_time = _chips[cp1]->getTime();
    if (_chips[cp2]->getTime() > sync_time)
        sync_time = _chips[cp2]->getTime();
    int tick1 = 0, tick2 = 0;
    while (_chips[cp1]->getTime() < sync_time) {
        tick1++;
        _chips[cp1]->tick();
    }
    while (_chips[cp2]->getTime() < sync_time + net_overhead) {
        tick2++;
        _chips[cp2]->tick();
    }
#ifdef NET_DEBUG_OUTPUT
    printf("Send a network request from Chip#%d to Chip#%d at %lu with %d overhead!\n",
            cp1, cp2, sync_time, net_overhead);
#endif
    _conn->issueNetReq(cp1, cp2, req.size_list[0], tick1, tick2, net_overhead);
    if (tick1 > tick2) 
        return tick1;
    return tick2;
}

int 
System::sendRowMv(Request& req)
{
    int tot_clks = 0;
    int n_ops = req.addr_list.size();
    for (int i = 0; i < n_ops; i+=2) {
        AddrT src_addr = req.addr_list[i],
              dst_addr = req.addr_list[i+1];
        int src_size  = req.size_list[i],
            dst_size  = req.size_list[i+1];
        int src_chip = 0, src_tile = 0, src_block = 0, src_row = 0, src_col = 0,
            dst_chip = 0, dst_tile = 0, dst_block = 0, dst_row = 0, dst_col = 0;

        getLocation(src_addr, src_chip, src_tile, src_block, src_row, src_col);
        getLocation(dst_addr, dst_chip, dst_tile, dst_block, dst_row, dst_col);
        req.setLocation(src_chip, src_tile, src_block, src_row, src_col);

        if ((src_chip != dst_chip) || (src_tile != dst_tile) || (src_block != dst_block))
            return -1;

        if ((src_col + src_size > _ncols) || (dst_col + dst_size > _ncols))
            return -1;

        Request rm_req(Request::Type::RowMv);

        rm_req.addAddr(src_addr, src_size);
        rm_req.addAddr(dst_addr, dst_size);
        rm_req.setLocation(src_chip, src_tile, src_block, src_row, -1);

        tot_clks++;
        bool res = _chips[src_chip]->receiveReq(rm_req);
        while (!res) {
            tot_clks++;
            _chips[src_chip]->tick();
            res = _chips[src_chip]->receiveReq(rm_req);
        }
    }
    return tot_clks;
}

int
System::sendColMv(Request& req)
{
    int tot_clks = 0;
    int n_ops = req.addr_list.size();
    for (int i = 0; i < n_ops; i+=2) {
        AddrT src_addr = req.addr_list[i],
              dst_addr = req.addr_list[i+1];
        int src_size  = req.size_list[i],
            dst_size  = req.size_list[i+1];
        int src_chip = 0, src_tile = 0, src_block = 0, src_row = 0, src_col = 0,
            dst_chip = 0, dst_tile = 0, dst_block = 0, dst_row = 0, dst_col = 0;

        getLocation(src_addr, src_chip, src_tile, src_block, src_row, src_col);
        getLocation(dst_addr, dst_chip, dst_tile, dst_block, dst_row, dst_col);
        req.setLocation(src_chip, src_tile, src_block, src_row, src_col);

        if ((src_chip != dst_chip) || (src_block != dst_block))
            return -1;

        if ((src_row + src_size > _nrows) || (dst_row + dst_size > _nrows))
            return -1;

        Request cm_req(Request::Type::ColMv);

        cm_req.addAddr(src_addr, src_size);
        cm_req.addAddr(dst_addr, dst_size);
        cm_req.setLocation(src_chip, src_tile, src_block, -1, src_col);

        tot_clks++;
        bool res = _chips[src_chip]->receiveReq(cm_req);
        while (!res) {
            tot_clks++;
            _chips[src_chip]->tick();
            res = _chips[src_chip]->receiveReq(cm_req);
        }
    }
    return tot_clks;
}

int
System::sendRowPIM(Request& req)
{
    int tot_clks = 0;
    int n_ops = req.addr_list.size();
    for (int i = 0; i < n_ops; i++) {
        AddrT src_addr = req.addr_list[i];
        int src_chip = 0, src_tile= 0, src_block= 0, src_row = 0, src_col = 0;

        getLocation(src_addr, src_chip, src_tile, src_block, src_row, src_col);
        req.setLocation(src_chip, src_tile, src_block, src_row, src_col);

        Request pim_req(req.type);

        pim_req.addAddr(src_addr, req.size_list[i]);
        pim_req.setLocation(src_chip, src_tile, src_block, src_row, -1);

        tot_clks++;
        bool res = _chips[src_chip]->receiveReq(pim_req);

        while (!res) {
            tot_clks++;
            _chips[src_chip]->tick();
            res = _chips[src_chip]->receiveReq(pim_req);
        }
    }
    return 0;
}

int
System::sendColPIM(Request& req)
{
    int tot_clks = 0;
    int n_ops = req.addr_list.size();
    for (int i = 0; i < n_ops; i+=2) {
       AddrT src_addr = req.addr_list[i];
        int src_chip = 0, src_tile= 0, src_block= 0, src_row = 0, src_col = 0;

        getLocation(src_addr, src_chip, src_tile, src_block, src_row, src_col);
        req.setLocation(src_chip, src_tile, src_block, src_row, src_col);

        Request pim_req(req.type);

        pim_req.addAddr(src_addr, req.size_list[i]);
        pim_req.setLocation(src_chip, src_tile, src_block, -1, src_col);

        tot_clks++;
        bool res = _chips[src_chip]->receiveReq(pim_req);
        while (!res) {
            tot_clks++;
            _chips[src_chip]->tick();
            res = _chips[src_chip]->receiveReq(pim_req);
        }
    }
    return 0;
}

int
System::sendRowBuffer(Request& req)
{
    int tot_clks = 0;
    int n_ops = req.addr_list.size();
    for (int i = 0; i < n_ops; i++) {
        AddrT src_addr = req.addr_list[i];
        int src_size  = req.size_list[i];
        int src_chip = 0, src_tile= 0, src_block= 0, src_row = 0, src_col = 0;

        getLocation(src_addr, src_chip, src_tile, src_block, src_row, src_col);
        req.setLocation(src_chip, src_tile, src_block, src_row, src_col);

        //DELETE printf("sendrowbuffer src %lu\n", src_addr);
        if (src_col + src_size > _ncols)
            return -1;
        Request buf_req(req.type);

        buf_req.addAddr(src_addr, src_size);
        buf_req.setLocation(src_chip, src_tile, src_block, src_row, -1);

        tot_clks++;
        bool res = _chips[src_chip]->receiveReq(buf_req);
        while (!res) {
            tot_clks++;
            _chips[src_chip]->tick();
            res = _chips[src_chip]->receiveReq(buf_req);
        }
    }
    return tot_clks;
}

int
System::sendColBuffer(Request& req)
{
    int tot_clks = 0;
    int n_ops = req.addr_list.size();
    for (int i = 0; i < n_ops; i++) {
        AddrT src_addr = req.addr_list[i];
        int src_size  = req.size_list[i];
        int src_chip = 0, src_tile= 0, src_block= 0, src_row = 0, src_col = 0;

        getLocation(src_addr, src_chip, src_tile, src_block, src_row, src_col);
        req.setLocation(src_chip, src_tile, src_block, src_row, src_col);

        //cout<<"sendcolbuffer src %lu\n"<< src_addr<<endl;
        //cout<<"sendcolbuffer size %lu\n"<< src_size<<endl;
        if (src_row + src_size > _nrows)
            return -1;

        Request buf_req(req.type);

        buf_req.addAddr(src_addr, src_size);
        buf_req.setLocation(src_chip, src_tile, src_block, -1, src_col);

        tot_clks++;
        bool res = _chips[src_chip]->receiveReq(buf_req);
        while (!res) {
            tot_clks++;
            _chips[src_chip]->tick();
            res = _chips[src_chip]->receiveReq(buf_req);
        }
    }
    return tot_clks;
}

int
System::sendPimReq(Request& req) 
{
    int return_value = 0;
    switch (req.type) {
        case Request::Type::RowMv:
            return_value =  sendRowMv(req);
            break;
        case Request::Type::ColMv:
            return_value =   sendColMv(req);
            break;
        case Request::Type::RowAdd:
        case Request::Type::RowSub:
        case Request::Type::RowMul:
        case Request::Type::RowDiv:
        case Request::Type::RowBitwise:
        case Request::Type::RowSearch:
            return_value =  sendRowPIM(req);
            break;
        case Request::Type::ColAdd:
        case Request::Type::ColSub:
        case Request::Type::ColMul:
        case Request::Type::ColDiv:
        case Request::Type::ColBitwise:
        case Request::Type::ColSearch: 
            return_value =   sendColPIM(req);
            break;
        default:
            cout << "Error: cannot handle non-PIM operations here!\n";
            break;
    }
    return return_value;
}

int
System::sendRequest(Request& req) 
{
#ifdef DEBUG_OUTPUT
    // std::cout << "The system is sending a request - " ;
#endif
    int ticks = 0;
    tot_reqs++;
    switch (req.type) {
        case Request::Type::Read:
        case Request::Type::Write:
            ticks = sendMoReq(req);
            break;
        case Request::Type::RowMv:
        case Request::Type::ColMv:
        case Request::Type::RowAdd:
        case Request::Type::ColAdd:
        case Request::Type::RowSub:
        case Request::Type::ColSub:
        case Request::Type::RowMul:
        case Request::Type::RowDiv:
        case Request::Type::ColMul:
        case Request::Type::ColDiv:
        case Request::Type::RowBitwise:
        case Request::Type::ColBitwise:
        case Request::Type::RowSearch:
        case Request::Type::ColSearch:
            ticks = sendPimReq(req);
            break;
        case Request::Type::RowBufferRead:
        case Request::Type::RowBufferWrite:
            ticks =  sendRowBuffer(req);
            break;
        case Request::Type::ColBufferRead:
        case Request::Type::ColBufferWrite:
            ticks =  sendColBuffer(req);
            break;
        case Request::Type::NetworkSend:
        case Request::Type::NetworkReceive:
#ifdef NET_DEBUG_OUTPUT
            printf("A request from library: %lu, %lu\n", req.addr_list[0], req.addr_list[1]);
#endif
            ticks = sendNetReq(req);
            break;
        case Request::Type::SystemRow2Row:
            ticks =  system_sendRow_receiveRow(req);
            break;
        case Request::Type::SystemRow2Col:
            ticks =  system_sendRow_receiveCol(req);
            break;
        case Request::Type::SystemCol2Row:
            ticks =  system_sendCol_receiveRow(req);
            break;
        case Request::Type::SystemCol2Col:
            ticks =  system_sendCol_receiveCol(req);
            break;
        default:
            cout << "[Error] unrecognized request!\n";
            break;
    }
#ifdef SCHED_DEBUG_OUTPUT
    printf("The system receives a Request#%lu: %s, ticks: %d\n", 
            tot_reqs, req.reqToStr().c_str(), ticks);
#endif
#ifdef DEBUG_OUTPUT
    // std::cout << req.reqToStr() << std::endl;
#endif
    if (ticks < 0) {
        std::cout << "Wrong Address!" << std::endl;
        exit(1);
    }

    vector<int> chips;
    for (int i = 0; i < _nchips; i++) {
        chips.push_back(i);
        while (!_chips[i]->isFinished())
            _chips[i]->tick();
    }
    sync(chips);

    return ticks;
}

void
System::sync(vector<int> chips)
{
    TimeT max_time = 0;
    for (int i : chips) {
        if (_chips[i]->getTime() > max_time) {
            max_time = _chips[i]->getTime();
        }
    }
    for (int i : chips) {
        while (_chips[i]->getTime() < max_time) {
            _chips[i]->tick();
        }
        _chips[i]->updateTime();
    }
}

void
System::finish()
{
    fprintf(rstFile, "\n############# Backend ##############\n");

    for (int i = 0; i < _nchips; i++) {
        while (!_chips[i]->isFinished())
            _chips[i]->tick();
        _chips[i]->outputStats(rstFile);
    }

    fprintf(rstFile, "\n############# Network #############\n");
    _conn->outputStat(rstFile);

    fprintf(rstFile, "\n############# Summary #############\n");
    for (int i = 0; i < _nchips; i++) {
        fprintf(rstFile, "Chip#%d has ticked %lu clocks\n", i, _chips[i]->getTime());
        fprintf(rstFile, "Chip#%d has consumed %.4lf nj energy\n", i, _chips[i]->getTotalEnergy());
    }
}

int 
System::system_sendRow_receiveRow(Request& req) {
    int tot_clks = 0;
    int n_ops = req.addr_list.size();
    for (int i = 0; i < n_ops; i+=2) {
        AddrT src_addr = req.addr_list[i],
              dst_addr = req.addr_list[i+1];
        int src_size  = req.size_list[i],
            dst_size  = req.size_list[i+1];

        int src_chip = 0, src_tile = 0, src_block = 0, src_row = 0, src_col = 0,
            dst_chip = 0, dst_tile = 0, dst_block = 0, dst_row = 0, dst_col = 0;

        getLocation(src_addr, src_chip, src_tile, src_block, src_row, src_col);
        getLocation(dst_addr, dst_chip, dst_tile, dst_block, dst_row, dst_col);
        req.setLocation(src_chip, src_tile, src_block, src_row, src_col);

        if ((src_col + src_size > _ncols) || (dst_col + dst_size > _ncols)) {
            return -1;
        }
            
        if (src_chip != dst_chip) {
#ifdef NET_DEBUG_OUTPUT
            printf("A request from RowMv: %lu, %lu\n", net_req.addr_list[0], net_req.addr_list[1]);
#endif
            Request buffer_read_req(Request::Type::RowBufferRead);
            buffer_read_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendRowBuffer(buffer_read_req);

            Request net_send_req(Request::Type::NetworkSend);
            net_send_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendNetReq(net_send_req);

            Request net_receive_req(Request::Type::NetworkReceive);
            net_receive_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendNetReq(net_receive_req);

            Request buffer_write_req(Request::Type::RowBufferWrite);
            buffer_write_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendRowBuffer(buffer_write_req);
        } else if ((src_tile != dst_tile) || (src_block != dst_block)) {
            Request buffer_read_req(Request::Type::RowBufferRead);
            buffer_read_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendRowBuffer(buffer_read_req);

            Request buffer_write_req(Request::Type::RowBufferWrite);
            buffer_write_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendRowBuffer(buffer_write_req);
        } else {
            Request row_mv_req(Request::Type::RowMv);
            row_mv_req.addAddr(src_addr, req.size_list[i]);
            row_mv_req.addAddr(dst_addr, req.size_list[i]);
            tot_clks += sendRowMv(row_mv_req);
        }
    }
    return tot_clks;
}

void System::example_1() {
        // FloatPIM are partitioned into Computing Units (PIM) and Storage Units
    // Matrixes are intially stored in the Storage Units.

    // Memory Hirechy: Block, Tile, Chip
    // Assume 1 Chip, 16 Tile, 256 Blocks, 1024 rows, 1024 columns
    // Assume data type is float-32

    AddrT storage_start_address =  (AddrT)_ncols *_nrows * _nblocks * _ntiles / 4 * 3; // use the last 3/4 for storage units
    AddrT pim_start_address = 0;

    std::vector<Request> requests;
    Request *request;
    request = new Request(Request::Type::SystemRow2Row);
    request->addAddr(storage_start_address, 2 * 32); // send A, B to the first row in block 0
    request->addAddr(pim_start_address, 2 * 32);
    request->addAddr(storage_start_address + 2 * 32, 2 * 32); // send C, D to the second row in block 0
    request->addAddr(pim_start_address + _ncols, 2 * 32);
    requests.push_back(*request);

    request = new Request(Request::Type::RowMul);
    request->addAddr(pim_start_address, 2*32); // Calculating temp1 = A * B
    request->addAddr(pim_start_address + _ncols, 2*32); // Calculating temp2 =  C * D
    requests.push_back(*request);

    // ****temp1****
    // ****temp2****
    request = new Request(Request::Type::RowBitwise);
    request->addAddr(pim_start_address + _ncols, 32); //Shift the second row
    requests.push_back(*request);

    // ****temp1****
    // ********temp2

    request = new Request(Request::Type::ColMv); //Move temp2 to the first row
    request->addAddr(pim_start_address , 20*32);
    request->addAddr(pim_start_address, 20*32);
    request->addAddr(pim_start_address +1 , 20*32);
    request->addAddr(pim_start_address +1, 20*32);
    requests.push_back(*request);

    // ****temp1temp2
    // ********temp2

    request = new Request(Request::Type::RowAdd); // Add temp1 and temp2
    request->addAddr(pim_start_address, 32*2);
    requests.push_back(*request);

    //****temp1temp2****S
    // ********temp2

    request = new Request(Request::Type::SystemRow2Row); // Send S back to storage units
    request->addAddr(pim_start_address, 32);
    request->addAddr(storage_start_address + _ncols, 32);
    requests.push_back(*request);

    for (unsigned int i = 0; i < requests.size(); i++)
        sendRequest(requests[i]);
}

void System::example_2() {
    // FloatPIM are partitioned into Computing Units (PIM) and Storage Units
    // Matrixes are intially stored in the Storage Units.

    // Memory Hirechy: Block, Tile, Chip
    // Assume 1 Chip, 16 Tile, 256 Blocks, 1024 rows, 1024 columns
    // Assume data type is float-32

    AddrT storage_start_address = (AddrT) _nrows * _ncols* _nblocks * _ntiles * 3 / 4; // use the last 3/4 for storage units
    AddrT pim_start_address = 0;

    std::vector<Request> requests;
    Request *request;
    request = new Request(Request::Type::SystemCol2Col);

    request->addAddr(storage_start_address, 32); // send A to the first row in block 0
    request->addAddr(pim_start_address, 32);
    //request->addAddr(storage_start_address + 1, 20*32); // send A to the first row in block 0
    //request->addAddr(pim_start_address + _ncols, 20*32);
    //request->addAddr(storage_start_address + 1, 20*32); // send A to the first row in block 0
    //request->addAddr(pim_start_address + 2*_ncols, 20*32);
    //request->addAddr(storage_start_address, 32); // send A to the first row in block 0
    //request->addAddr(pim_start_address + 2*_ncols, 32);
    //request->addAddr(storage_start_address, 32); // send A to the first row in block 0
    //request->addAddr(pim_start_address + 3*_ncols, 32);
    //request->addAddr(storage_start_address + 1, 32); // send B to the first row in block 0
    //request->addAddr(pim_start_address + 32, 32);
    /*request->addAddr(storage_start_address + 2 , 32); // send C to the second row in block 0
    request->addAddr(pim_start_address + _ncols, 32);
    request->addAddr(storage_start_address + 3, 32); // send D to the second row in block 0
    request->addAddr(pim_start_address + _ncols + 32, 32);
*/
    requests.push_back(*request);


    request = new Request(Request::Type::RowMul);
    request->addAddr(pim_start_address, 32*2); // Calculating temp1 = A * B
    request->addAddr(pim_start_address+ _ncols, _ncols); // Calculating temp2 =  C * D
    requests.push_back(*request);

    // ****temp1****
    // ****temp2****
    request = new Request(Request::Type::RowBitwise);
    request->addAddr(pim_start_address + _ncols, 32); //Shift the second row
    requests.push_back(*request);

    // ****temp1****
    // ********temp2

    request = new Request(Request::Type::RowMv); //Move temp2 to the first row
    request->addAddr(pim_start_address + _ncols, 32); 
    request->addAddr(pim_start_address, 32);
    requests.push_back(*request);

    // ****temp1temp2
    // ********temp2

    request = new Request(Request::Type::RowAdd); // Add temp1 and temp2
    request->addAddr(pim_start_address, 2*32);
    requests.push_back(*request);

    //****temp1temp2****S
    // ********temp2

    request = new Request(Request::Type::SystemRow2Col); // Send S back to storage units
    request->addAddr(pim_start_address, 32);
    request->addAddr(storage_start_address + 4, 32);
    requests.push_back(*request);

    for (unsigned int i = 0; i < requests.size(); i++)
        sendRequest(requests[i]);
}

/***************************************************************/
/* Do not modify the above code.

   Begin your code here 	  			       */
/***************************************************************/

int 
System::system_sendRow_receiveCol(Request& req) {
    int tot_clks = 0;
    int n_ops = req.addr_list.size();
    for (int i = 0; i < n_ops; i+=2) {
        AddrT src_addr = req.addr_list[i],
              dst_addr = req.addr_list[i+1];
        int src_size  = req.size_list[i],
            dst_size  = req.size_list[i+1];

        int src_chip = 0, src_tile = 0, src_block = 0, src_row = 0, src_col = 0,
            dst_chip = 0, dst_tile = 0, dst_block = 0, dst_row = 0, dst_col = 0;

        getLocation(src_addr, src_chip, src_tile, src_block, src_row, src_col);
        getLocation(dst_addr, dst_chip, dst_tile, dst_block, dst_row, dst_col);
        req.setLocation(src_chip, src_tile, src_block, src_row, src_col);

        if ((src_col+ src_size > _ncols) || (dst_row + dst_size > _nrows)) {
            printf("%d %d %d %d \n", src_col, src_size , dst_row , dst_size);
            return -1;
        }

        if (src_chip != dst_chip) {
#ifdef NET_DEBUG_OUTPUT
            printf("A request from Row2ColMv: %lu, %lu\n", net_req.addr_list[0], net_req.addr_list[1]);
#endif
            Request buffer_read_req(Request::Type::RowBufferRead);
            buffer_read_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendRowBuffer(buffer_read_req);

            Request net_send_req(Request::Type::NetworkSend);
            net_send_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendNetReq(net_send_req);

            Request net_receive_req(Request::Type::NetworkReceive);
            net_receive_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendNetReq(net_receive_req);

            Request buffer_write_req(Request::Type::ColBufferWrite);
            buffer_write_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendColBuffer(buffer_write_req);
        } else{
            //DELETEprintf("src %lu\n", src_addr);
            Request buffer_read_req(Request::Type::RowBufferRead);
            buffer_read_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendRowBuffer(buffer_read_req);

            //DELETEprintf("dst %lu\n", dst_addr);
            Request buffer_write_req(Request::Type::ColBufferWrite);
            buffer_write_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendColBuffer(buffer_write_req);
        }
    }
    return tot_clks;
}
    
int 
System::system_sendCol_receiveRow(Request& req) {
    int tot_clks = 0;
    int n_ops = req.addr_list.size();
    for (int i = 0; i < n_ops; i+=2) {
        AddrT src_addr = req.addr_list[i],
              dst_addr = req.addr_list[i+1];
        int src_size  = req.size_list[i],
            dst_size  = req.size_list[i+1];

        int src_chip = 0, src_tile = 0, src_block = 0, src_row = 0, src_col = 0,
            dst_chip = 0, dst_tile = 0, dst_block = 0, dst_row = 0, dst_col = 0;

        getLocation(src_addr, src_chip, src_tile, src_block, src_row, src_col);
        getLocation(dst_addr, dst_chip, dst_tile, dst_block, dst_row, dst_col);
        req.setLocation(src_chip, src_tile, src_block, src_row, src_col);

        if ((src_row+ src_size > _nrows) || (dst_col + dst_size > _ncols)) {
            return -1;
        }

        if (src_chip != dst_chip) {
#ifdef NET_DEBUG_OUTPUT
            printf("A request from Row2ColMv: %lu, %lu\n", net_req.addr_list[0], net_req.addr_list[1]);
#endif
            Request buffer_read_req(Request::Type::ColBufferRead);
            buffer_read_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendColBuffer(buffer_read_req);

            Request net_send_req(Request::Type::NetworkSend);
            net_send_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendNetReq(net_send_req);

            Request net_receive_req(Request::Type::NetworkReceive);
            net_receive_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendNetReq(net_receive_req);

            Request buffer_write_req(Request::Type::RowBufferWrite);
            buffer_write_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendRowBuffer(buffer_write_req);
        } else{
            Request buffer_read_req(Request::Type::ColBufferRead);
            buffer_read_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendColBuffer(buffer_read_req);

            Request buffer_write_req(Request::Type::RowBufferWrite);
            buffer_write_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendRowBuffer(buffer_write_req);
        }
    }
    return tot_clks;
}
    
int 
System::system_sendCol_receiveCol(Request& req) {
    int tot_clks = 0;
    int n_ops = req.addr_list.size();
    for (int i = 0; i < n_ops; i+=2) {
        AddrT src_addr = req.addr_list[i],
              dst_addr = req.addr_list[i+1];
        int src_size  = req.size_list[i],
            dst_size  = req.size_list[i+1];

        int src_chip = 0, src_tile = 0, src_block = 0, src_row = 0, src_col = 0,
            dst_chip = 0, dst_tile = 0, dst_block = 0, dst_row = 0, dst_col = 0;

        getLocation(src_addr, src_chip, src_tile, src_block, src_row, src_col);
        getLocation(dst_addr, dst_chip, dst_tile, dst_block, dst_row, dst_col);
        req.setLocation(src_chip, src_tile, src_block, src_row, src_col);
            //cout<<"sendcolbuffer src %lu\n"<< src_row<<endl;
            //cout<<"sendcolbuffer size %lu\n"<< src_size<<endl;

        if ((src_row+ src_size > _nrows) || (dst_row + dst_size > _nrows)) {
            return -1;
        }

        if (src_chip != dst_chip) {
#ifdef NET_DEBUG_OUTPUT
            printf("A request from Col2ColMv: %lu, %lu\n", net_req.addr_list[0], net_req.addr_list[1]);
#endif
            Request buffer_read_req(Request::Type::ColBufferRead);
            buffer_read_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendColBuffer(buffer_read_req);

            Request net_send_req(Request::Type::NetworkSend);
            net_send_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendNetReq(net_send_req);

            Request net_receive_req(Request::Type::NetworkReceive);
            net_receive_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendNetReq(net_receive_req);

            Request buffer_write_req(Request::Type::ColBufferWrite);
            buffer_write_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendColBuffer(buffer_write_req);
        } else if ((src_tile != dst_tile) || (src_block != dst_block)) {
            Request buffer_read_req(Request::Type::ColBufferRead);
            buffer_read_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendColBuffer(buffer_read_req);

            Request buffer_write_req(Request::Type::ColBufferWrite);
            buffer_write_req.addAddr(dst_addr, req.size_list[i+1]);
            tot_clks += sendColBuffer(buffer_write_req);
        } else {
            Request col_mv_req(Request::Type::ColMv);
            col_mv_req.addAddr(src_addr, req.size_list[i]);
            col_mv_req.addAddr(dst_addr, req.size_list[i]);
            tot_clks += sendColMv(col_mv_req);
        }
    }
    return tot_clks;
}

void System::matrix_mul_area_optimized(int A_row, int A_col, int B_row, int B_col) 
{
    AddrT storage_start_address =  (AddrT)_ncols *_nrows * _nblocks * _ntiles / 4 * 3; // use the last 3/4 for storage units
    AddrT pim_start_address = 0;

    int mul1_p = 0;
    int height = 40;
    int a_p    = mul1_p;
    int b_p    = 2;

    AddrT data_a_p = 0;
    AddrT data_b_p = 200*_ncols*_nrows;
    AddrT pim_p = 0;
    AddrT sum_p = 4000*_ncols*_nrows; //TODO


    pim_p  = pim_start_address;
    data_a_p = storage_start_address;
    data_b_p = storage_start_address + (AddrT) 2*256*1024*1024*32;


   	std::vector<Request> requests;
   	Request *request;
   	//Loop to traverse A
   	//for (int a_ii = 0; a_ii < A_row;a_ii++){
   	for (int a_ii = 0; a_ii < 10;a_ii++){
//------------------------transmit one A row to the block 0------------------------------------------//
    	request = new Request(Request::Type::SystemCol2Col);
    	for (int ii = 0; ii < 2; ii++){// no of blocks used
    		request->addAddr(data_a_p + (AddrT)(a_ii*2  + ii), 20*32);
    		request->addAddr(pim_p  + (AddrT) (a_p + ii*2) ,20*32);
    	}
    	requests.push_back(*request);
                 //-----------Shift A ------------//
    	request = new Request(Request::Type::ColBitwise);
    	for (int no_bit_byte= 0; no_bit_byte < 32;no_bit_byte++){//i: index of current A row
    		request->addAddr(pim_p  + (AddrT)(32 + no_bit_byte) ,20);//In real case, we need to define the location of each A col
    	}
    	requests.push_back(*request);
                 //-----------ColMv A to get complete one colum------------//
    	request = new Request(Request::Type::ColMv);
    	request->addAddr(pim_p  + (AddrT) (a_p + 2) ,20*32);
    	request->addAddr(pim_p  + (AddrT) (a_p) ,20*32);
    	requests.push_back(*request);

    	//Loop to traverse B
    	for (int b_ii = 0; b_ii <B_col;b_ii++){
//------------------------transmit one B col to the block 0------------------------------------------//
    		request = new Request(Request::Type::SystemCol2Col);
    		for (int ii = 0; ii < 2; ii++){// no of blocks used
    			request->addAddr(data_b_p + (AddrT)(b_ii*2  + ii), 20*32);
    			request->addAddr(pim_p  + (AddrT) (b_p + ii*2) ,20*32);
    		}
    		requests.push_back(*request);
                 //-----------Shift B ------------//
    		request = new Request(Request::Type::ColBitwise);
    		for (int no_bit_byte= 0; no_bit_byte < 32;no_bit_byte++){//i: index of current A row
    			request->addAddr(pim_p  + (AddrT)(3*32 + no_bit_byte) ,20);//In real case, we need to define the location of each A col
    		}
    		requests.push_back(*request);
                 //-----------ColMv B to get complete one colum------------//
    		request = new Request(Request::Type::ColMv);
    		request->addAddr(pim_p  + (AddrT) (b_p + 2) ,20*32);
    		request->addAddr(pim_p  + (AddrT) (b_p) ,20*32);
    		requests.push_back(*request);
//------------------------Calculation------------------------------------------//
    		//loop to do multiplication
    		request = new Request(Request::Type::RowMul);
    		for (int m_i = 0; m_i <height;m_i++){
    			request->addAddr(pim_p  + (AddrT) (m_i*_ncols) ,2*32); //The results is stored at sum_p
    		}
    		requests.push_back(*request);
    		//loop to do addition
    		for (int add_i = 1; add_i <height;add_i++){
    			request = new Request(Request::Type::ColAdd);
    			request->addAddr(pim_p,2*32);//The results will be stored at the end of this col
    			requests.push_back(*request);
    		}

    		//send sum back to storage unit
    		request = new Request(Request::Type::SystemRow2Row);
    		request->addAddr(pim_p  + (AddrT) (2*height -1 ) ,32);//The results will be stored at the end of this col
    		request->addAddr(sum_p ,32);//The results will be stored at the end of this col
    		requests.push_back(*request);
    		//update sum_p to the next block
    		sum_p = sum_p + (AddrT)  _ncols*_nrows;
    		//issure request
    		for (unsigned int i = 0; i < requests.size(); i++){
    			sendRequest(requests[i]);
    		}
    		std::vector<Request>().swap(requests);

    	}//loop to traverse B
    }//loop to traverse A

}

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
    int b_p    = mul2_p;
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
    		}
    	}
    }

    //Fill block-col of B to each PIM

    for (int n_b_col = 0; n_b_col < B_col * B_col/height; n_b_col++){//i: index of current A row
    	for (int n_blk = 0; n_blk < no_block_same_b; n_blk++){// no of blocks used
    		for (int ii = 0; ii < B_col/20; ii++){// no of blocks used
    			request->addAddr(data_b_p + (AddrT)(n_b_col *2 + ii), 20*32);
    			request->addAddr(pim_b_p  + (AddrT) (n_blk * no_block_same_a * _ncols*_nrows + (n_b_col/b_width)) *_ncols*_nrows + b_p + n_b_col%b_width + ii*2 ,20*32);
    		}
    	}
    }
    requests.push_back(*request);

    for (unsigned int i = 0; i < requests.size(); i++){
        sendRequest(requests[i]);
    }
   	std::vector<Request>().swap(requests);

//shift
    request = new Request(Request::Type::ColBitwise);
    for (int n_b_blk= 0; n_b_blk < no_block;n_b_blk++){//i: index of current A row
    	for (int no_bit_byte= 0; no_bit_byte < 32;no_bit_byte++){//i: index of current A row
    		request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + (a_p + 2 + no_bit_byte)*32) ,20);
    		request->addAddr(pim_b_p  + (AddrT) (n_b_blk * _ncols*_nrows + (b_p + 2 + no_bit_byte)*32) ,20);
    	}
    }
    requests.push_back(*request);

    for (unsigned int i = 0; i < requests.size(); i++){
      sendRequest(requests[i]);
    }
   	std::vector<Request>().swap(requests);

//ColMv
    request = new Request(Request::Type::ColMv);
    for (int n_b_blk= 0; n_b_blk < no_block;n_b_blk++){//i: index of current A row
    		request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + (a_p + 2)) ,20);
    		request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + (a_p + 2)) ,20);
    		request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + (b_p) ) ,20);
    		request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + (b_p) ) ,20);
    }
    requests.push_back(*request);

    for (unsigned int i = 0; i < requests.size(); i++){
      sendRequest(requests[i]);
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
       sendRequest(requests[i]);
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
    		sendRequest(requests[i]);
    	}
    	std::vector<Request>().swap(requests);
    }



//Send back
    request = new Request(Request::Type::SystemRow2Row);
    for (int n_b_blk= 0; n_b_blk < no_block;n_b_blk++){
    	request->addAddr(pim_a_p  + (AddrT) (n_b_blk * _ncols*_nrows + 2*height -1 ) ,32);//The results will be stored at the end of this col
    	request->addAddr(data_a_p  + (AddrT) (n_b_blk * _ncols*_nrows ) ,32);//The results will be stored at the end of this col
    }
    requests.push_back(*request);
    for (unsigned int i = 0; i < requests.size(); i++){
    	//sendRequest(requests[i]);
    }
    std::vector<Request>().swap(requests);






}

void System::matrix_mul_balanced(int A_row, int A_col, int B_row, int B_col) 
{
/*Write your code here*/
}
