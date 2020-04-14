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

        printf("sendrowbuffer src %lu\n", src_addr);
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

        printf("sendcolbuffer src %lu\n", src_addr);
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

    request = new Request(Request::Type::RowMv); //Move temp2 to the first row
    request->addAddr(pim_start_address + _ncols, 32); 
    request->addAddr(pim_start_address, 32);
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
    request = new Request(Request::Type::SystemCol2Row);

    request->addAddr(storage_start_address, 32); // send A to the first row in block 0
    request->addAddr(pim_start_address, 32);
    request->addAddr(storage_start_address + 1, 32); // send B to the first row in block 0
    request->addAddr(pim_start_address + 32, 32);
    request->addAddr(storage_start_address + 2 , 32); // send C to the second row in block 0
    request->addAddr(pim_start_address + _ncols, 32);
    request->addAddr(storage_start_address + 3, 32); // send D to the second row in block 0
    request->addAddr(pim_start_address + _ncols + 32, 32);
    requests.push_back(*request);


    request = new Request(Request::Type::RowMul);
    request->addAddr(pim_start_address, 32*2); // Calculating temp1 = A * B
    request->addAddr(pim_start_address + _ncols, _ncols); // Calculating temp2 =  C * D
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
            printf("src %lu\n", src_addr);
            Request buffer_read_req(Request::Type::RowBufferRead);
            buffer_read_req.addAddr(src_addr, req.size_list[i]);
            tot_clks += sendRowBuffer(buffer_read_req);

            printf("dst %lu\n", dst_addr);
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
/*Write your code here*/
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
    int a_p    = 2;
    int b_p    = 12;
    int ps_p   = 22;
    int no_a   = 10;     // Number of row in block which are occupied by A
    int no_b   = 10;     // Number of row in block which are occupied by B
    int block_row = 32; // Number of data can be stored in one row of block
    int no_block = ceil((float)A_col / (float)block_row);
    int a_sb_y = ceil((float)A_col/(float) no_a);


    AddrT data_a_p = 0;
    AddrT data_b_p = 0;
    AddrT pim_a_p = 0;
    AddrT pim_b_p = 0;
    AddrT sum_p = 4000*_ncols*_nrows; //TODO

    data_a_p = storage_start_address;
    pim_a_p  = pim_start_address;

   	std::vector<Request> requests;
   	Request *request;

    //Fill all A to PIM unit
    for (int a_y = 0; a_y < a_sb_y; a_y++){// no of sub block in y direction
    	//Move a no_a rows of A to 32 blocks, each block has no_a*32 data.
    	if(a_y == a_sb_y -1){
    		request = new Request(Request::Type::SystemRow2Col);
    		for (int i = 0; i < no_block; i++){//i: index of the current block
    			for (int j = 0; j < A_col - a_y*no_a; j++){//i: index of the row in current block{
    				request->addAddr(data_a_p + (i*no_a+j)*_ncols ,block_row*32);
    				request->addAddr(pim_a_p + a_p  + j + i*_ncols*_nrows, block_row*32);
    			}
    		}

    	}else{
    		request = new Request(Request::Type::SystemRow2Col);
    		for (int i = 0; i < no_block; i++){//i: index of the current block
    			for (int j = 0; j < no_a; j++){//i: index of the row in current block{
    				request->addAddr(data_a_p + (i*no_a+j)*_ncols ,block_row*32);
    				request->addAddr(pim_a_p + a_p  + j + i*_ncols*_nrows, block_row*32);
    			}
    		}
    		//update data_a_p and pim_a_p
    		data_a_p =+ (AddrT) no_block*no_a*block_row*32;
    		pim_a_p  =+ (AddrT) A_col/block_row*_ncols*_nrows;
    	}
    }

    //Move a no_b cols of B to 32 blocks, each block has no_b*32 data.
    data_b_p = storage_start_address + A_col/block_row*no_a*block_row*32;
    pim_b_p  = pim_start_address;
    for (int i = 0; i < no_block; i++){//i: index of the current block
    	for (int j = 0; j < no_b; j++){//i: index of the row in current block{
    		request->addAddr(data_b_p + (i*no_b+j)*_ncols ,block_row*32);
    		request->addAddr(pim_b_p + b_p  + j + i*_ncols*_nrows, block_row*32);
    	}
	}
    //update data_b_p and pim_b_p
    data_b_p =+ (AddrT) B_col/block_row*no_b;
    pim_b_p  =+ (AddrT) B_col/block_row*_ncols*_nrows;

    requests.push_back(*request);


    //Start calculation, all the loaded block will start to this at the same time
    for (int ii = 0; ii <no_a;ii++){//ii: index of current A col
    	//Move col ii  of A to mul1
    	request = new Request(Request::Type::ColMv);
    	for (int i = 0; i < no_block; i++){//i: index of the current block
    		request->addAddr(pim_start_address + i*_ncols*_nrows + a_p + ii, 32*32);
    		request->addAddr(pim_start_address + i*_ncols*_nrows + mul1_p + ii, 32*32);
    		request->addAddr(pim_start_address + i*_ncols*_nrows + b_p, 32*32);
    		request->addAddr(pim_start_address + i*_ncols*_nrows + mul2_p, 32*32);
    	}
    	requests.push_back(*request);

    	//Move col jj of B to mul2
    	for (int jj = 0; jj<no_b; jj++){//jj: index of the current B col
    		request = new Request(Request::Type::ColMv);
    		for (int i = 0; i < (B_row/block_row); i++){//i: index of the current block
    			request->addAddr(pim_start_address + i*_ncols*_nrows + b_p + jj, 32*32);
    			request->addAddr(pim_start_address + i*_ncols*_nrows + mul2_p, 32*32);
    		}
    		requests.push_back(*request);
            //multiply mul1 and mul2, store them to ps
    		request = new Request(Request::Type::RowMul);
    		for (int i = 0; i < no_block; i++){//i: index of the current block
    			for (int j = 0; j < block_row; j++){//j: do multiplication for j row of i block
    				request->addAddr(pim_start_address + i*_ncols*_nrows + j*_ncols, 2*32);
    				request->addAddr(pim_start_address + i*_ncols*_nrows + j*_ncols + ps_p + jj, 32);
    			}
    		}
    		requests.push_back(*request);
    	}

    	//Send partial sum to another block for addition
    	request = new Request(Request::Type::SystemCol2Col);
    	for (int i = 0; i < no_block; i++){//i: index of the set of partial sum/ equal to number of block
    		for (int j = 0; j < no_a; j++){//j: index of the col in current block{
    			request->addAddr(pim_start_address+ i*_ncols*_nrows+ ps_p + j ,32*32);
    			request->addAddr(sum_p + (i/2)*_ncols*_nrows+ no_a*(i%2) + j ,32*32);
    		}
    	}
    	requests.push_back(*request);

    	//Do addition for results for each block
    	for (int kk = 1; kk < block_row; kk++){//i: index of the set of partial sum// equal to number of block
    		//Move the next number to be added to the second row
    		request = new Request(Request::Type::RowMv);
    		for (int i = 0; i < no_block/2; i++){//i: index of the set of partial sum// equal to number of block
    			request->addAddr(sum_p + i*_ncols*_nrows + kk*_ncols,no_a*2*32);
    			request->addAddr(sum_p + i*_ncols*_nrows + _ncols,no_a*2*32);
    		}
    		requests.push_back(*request);

    		//Add first two rows and store the results to the first/second rows [20:29]
    		request = new Request(Request::Type::ColAdd);
    		for (int i = 0; i < no_block/2; i++){//i: index of the set of partial sum// equal to number of block
    			for (int j = 0; j < no_a*2; j++){//j: index of the col in current block{
    				request->addAddr(sum_p + i*_ncols*_nrows + j ,2*32);
    				request->addAddr(sum_p + i*_ncols*_nrows + no_a + (j/10)*_ncols + (j%10) ,2*32);
    			}
    		}
    		requests.push_back(*request);

    		//Move the intermedia results to the begining of the first row
    		request = new Request(Request::Type::RowBitwise);
    		for (int i = 0; i < no_block/2; i++){//i: index of the set of partial sum// equal to number of block
    			request->addAddr(sum_p + i*_ncols*_nrows + no_a,no_a*32);
    			request->addAddr(sum_p + i*_ncols*_nrows ,no_a*32);
    		}
    		requests.push_back(*request);
    		request = new Request(Request::Type::RowMv);
    		for (int i = 0; i < no_block/2; i++){//i: index of the set of partial sum// equal to number of block
    			request->addAddr(sum_p + i*_ncols*_nrows + _ncols + no_a,no_a*32);
    			request->addAddr(sum_p + i*_ncols*_nrows + no_a*2,no_a*32);
    		}
    		requests.push_back(*request);

    	}
    	//Do addition for all block_row blocks
    	//Move all the ps to the first block
    	request = new Request(Request::Type::SystemRow2Row);
    	for (int i = 1; i < no_block/2; i++){//i: index of the current block
    		request->addAddr(sum_p + i*_ncols*_nrows, no_a*2*32);
    		request->addAddr(sum_p + i*_ncols ,no_a*2*32);
    	}
    	requests.push_back(*request);

    	//Do addition
    	for (int kk = 1; kk < block_row/2; kk++){//i: index of the set of partial sum// equal to number of block
    		//Move the next number to be added to the second row
    		request = new Request(Request::Type::RowMv);
    		request->addAddr(sum_p + kk*_ncols,no_a*2*32);
    		request->addAddr(sum_p + _ncols,no_a*2*32);
    		requests.push_back(*request);

    		//Add first two rows and store the results to the first row [10:19]
    		request = new Request(Request::Type::ColAdd);
    		for (int j = 0; j < no_a*2; j++){//j: index of the col in current block{
    			request->addAddr(sum_p + j ,2*32);
   				request->addAddr(sum_p + no_a + (j/10)*_ncols + (j%10) ,2*32);
    		}
    		requests.push_back(*request);

    		//Move the intermedia results to the begining of the first row
    		request = new Request(Request::Type::RowBitwise);
    		request->addAddr(sum_p + no_a,no_a*32);
    		request->addAddr(sum_p , no_a*32);
    		requests.push_back(*request);

    		request = new Request(Request::Type::RowMv);
    		request->addAddr(sum_p + _ncols + no_a,no_a*32);
    		request->addAddr(sum_p + no_a*2,no_a*32);
    		requests.push_back(*request);
    	}
    		request = new Request(Request::Type::RowMv);
    		request->addAddr(sum_p ,no_a*32);
    		request->addAddr(sum_p + _ncols + no_a,no_a*32);
    		requests.push_back(*request);

    		request = new Request(Request::Type::ColAdd);
    		for (int i = 0; i < no_a; i++){//j: index of the col in current block{
    			request->addAddr(sum_p + no_a ,2*32);
    			request->addAddr(sum_p ,no_a*32);
    		}
    		requests.push_back(*request);

    	//Store the 10 result back to storage unit
    	request = new Request(Request::Type::SystemRow2Row);
    	request->addAddr(sum_p, no_a*32);
    	request->addAddr(data_a_p + ii*_ncols ,no_a*32);
    	requests.push_back(*request);


    }

    for (unsigned int i = 0; i < requests.size(); i++)
        sendRequest(requests[i]);
}

void System::matrix_mul_balanced(int A_row, int A_col, int B_row, int B_col) 
{
/*Write your code here*/
}
