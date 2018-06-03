--author @qiuyukun

local floor = math.floor
local abs   = math.abs
local max   = math.max
local min   = math.min

--convert the frg index to zero
local convert_index_to_0 = true

local  IKCP_RTO_NDL     = 30 -- no delay min rto
local  IKCP_RTO_MIN     = 100 -- normal min rto
local  IKCP_RTO_DEF     = 200
local  IKCP_RTO_MAX     = 60000
local  IKCP_CMD_PUSH    = 81 -- cmd: push data
local  IKCP_CMD_ACK     = 82 -- cmd: ack
local  IKCP_CMD_WASK    = 83 -- cmd: window probe (ask)
local  IKCP_CMD_WINS    = 84 -- cmd: window size (tell)
local  IKCP_ASK_SEND    = 1 -- need to send IKCP_CMD_WASK
local  IKCP_ASK_TELL    = 2 -- need to send IKCP_CMD_WINS
local  IKCP_WND_SND     = 32
local  IKCP_WND_RCV     = 32
local  IKCP_MTU_DEF     = 1024
local  IKCP_ACK_FAST    = 3
local  IKCP_INTERVAL    = 100
local  IKCP_OVERHEAD    = 24
local  IKCP_DEADLINK    = 10
local  IKCP_THRESH_INIT = 2
local  IKCP_THRESH_MIN  = 2
local  IKCP_PROBE_INIT  = 7000 -- 7 secs to probe window size
local  IKCP_PROBE_LIMIT = 120000 -- up to 120 secs to probe window

function clone(class)
    local lookup_table = {}
    local function copyObj( object )
        if type( object ) ~= "table" then
            return object
        elseif lookup_table[object] then
            return lookup_table[object]
        end

        local new_table = {}
        lookup_table[object] = new_table
        for key, value in pairs( object ) do
            new_table[copyObj( key )] = copyObj( value )
        end
        return setmetatable( new_table, getmetatable( object ) )
    end
    return copyObj( object )
end

local function bound(a,b,c)
    return min(max(a,b),c)
end

local function append(p, src)
    local t = {}
    for k,v in pairs(p) do
        table.insert(t, v)
    end

    for k,v in pairs(src) do
        table.insert(t, v)
    end
    return t
end

local function slice(p, start, stop)
    local t = {}
    local i = start
    for i = start, stop do
        table.insert(t, p[i])
    end
    return t
end

local function copy_array(src, src_idx, dest, dest_idx, length)
    for i = 1, length do
        dest[dest_idx + i] = src[src_idx + i]
    end
end

local function ikcp_encode8u(p, offset, c)
    p[1 + offset] = c
    return 1
end

-- decode 8 bits unsigned int
local function ikcp_decode8u(p, offset)
    return 1, p[1 + offset]
end

-- encode 16 bits unsigned int (lsb)
local function ikcp_encode16u(p, offset, w)
    p[1 + offset] = 255 & w >> 0
    p[2 + offset] = 255 & w >> 8
    return 2
end

-- decode 16 bits unsigned int (lsb)
local function ikcp_decode16u(p, offset)
    local result = 0
    result = result | p[1 + offset]
    result = result | p[2 + offset] << 8
    return 2, result
end

-- encode 32 bits unsigned int (lsb)
local function ikcp_encode32u(p, offset, l)
    p[1 + offset] = 255 & l >> 0
    p[2 + offset] = 255 & l >> 8
    p[3 + offset] = 255 & l >> 16
    p[4 + offset] = 255 & l >> 24
    return 4
end

-- decode 32 bits unsigned int (lsb)
local function ikcp_decode32u(p, offset)
    local result = 0
    result = result | p[1 + offset]
    result = result | p[2 + offset] << 8
    result = result | p[3 + offset] << 16
    result = result | p[4 + offset] << 24
    return 4, result
end


local segment = {
    conv = 0,
    cmd = 0,
    frg = 1,
    wnd = 0,
    ts = 0,
    sn = 0,
    una = 0,
    resendts = 0,
    rto = 0,
    fastack = 0,
    xmit = 0,
    data = {},

    encode = function(self, ptr, offset)
        local offset_ = offset
        offset = offset + ikcp_encode32u(ptr, offset, self.conv)
        offset = offset + ikcp_encode8u(ptr, offset, self.cmd)
        if convert_index_to_0 then
            offset = offset + ikcp_encode8u(ptr, offset, self.frg - 1)
        else
            offset = offset + ikcp_encode8u(ptr, offset, self.frg)
        end
        offset = offset + ikcp_encode16u(ptr, offset, self.wnd)
        offset = offset + ikcp_encode32u(ptr, offset, self.ts)
        offset = offset + ikcp_encode32u(ptr, offset, self.sn)
        offset = offset + ikcp_encode32u(ptr, offset, self.una)
        offset = offset + ikcp_encode32u(ptr, offset, #self.data)
        return offset - offset_
    end
}

local function new_segment()
    return clone(segment)
end

local kcp = {
    -- kcp members.
    conv = 0,

    mtu = 0,
    mss = 0,
    state = 0,
    snd_una = 0,
    snd_nxt = 0,
    rcv_nxt = 0,
    ts_recent = 0,
    ts_lastack = 0,
    ssthresh = 0,
    rx_rttval = 0,
    rx_srtt = 0,
    rx_rto = 0,
    rx_minrto = 0,
    snd_wnd = 0,
    rcv_wnd = 0,
    rmt_wnd = 0,
    cwnd = 0,
    probe = 0,
    current = 0,
    interval = 0,
    ts_flush = 0,
    xmit = 0,
    nodelay = 0,
    updated = 0,
    ts_probe = 0,
    probe_wait = 0,
    dead_link = 0,
    incr = 0,

    snd_queue = {},
    rcv_queue = {},
    snd_buf = {},
    rcv_buf = {},
    acklist = {},

    buffer,
    fastresend = 0,
    nocwnd = 0,

    logmask = 0,
    --buffer, size
    output,

    -- create a new kcp control object, 'conv' must equal in two endpoint
	-- from the same connection.


    -- check the size of next message in the recv queue
    peek_size = function(self)
        if 0 == #self.rcv_queue then
            return -1
        end

        local seq = self.rcv_queue[1]

        if 1 == seq.frg then
            return #seq.data
        end

        if seq.frg > #self.rcv_queue then
            return -1
        end

        local length = 0

        for k,v in pairs(self.rcv_queue) do
            length = length + #v.data
            if 1 == v.frg then
                break
            end
        end
        return length
    end,

    -- user/upper level recv: returns size, returns below zero for EAGAIN
    Recv = function(self, buffer)
        if 0 == #self.rcv_queue then
            return -1
        end

        local peek_size = self:peek_size()
		if peek_size < 0 then
			return -2
        end

		-- if peekSize > #buffer then
		-- 	return -3
        -- end

		local fast_recover = false
		if #self.rcv_queue >= self.rcv_wnd then
			fast_recover = true
        end

		-- merge fragment.
		local count = 0
		local n = 0

        for k,v in pairs(self.rcv_queue) do
            copy_array(v.data, 0, buffer, n, #v.data)
			n = n + #v.data
			count = count + 1
			if 1 == v.frg then
				break
            end
        end

		if count > 0 then
			self.rcv_queue = slice(self.rcv_queue, count + 1, #self.rcv_queue)
        end

		-- move available data from rcv_buf -> rcv_queue
		count = 0
        for k,v in pairs(self.rcv_buf) do
            if v.sn == self.rcv_nxt and #self.rcv_queue < self.rcv_wnd then
                table.insert(self.rcv_queue, v)
				self.rcv_nxt = self.rcv_nxt + 1
				count = count + 1
			else
				break
            end
        end

		if count > 0 then
			self.rcv_buf = slice(self.rcv_buf, count + 1, #self.rcv_buf)
        end

		-- fast recover
		if #self.rcv_queue < self.rcv_wnd and fast_recover then
			self.probe = self.probe | IKCP_ASK_TELL
        end
		return n
    end,

    -- user/upper level send, returns below zero for error
    Send = function(self, bytes, index, length)
        if 0 == #bytes then
            return -1
        end

        if length == 0 then
            return -1
        end

        local count = 0

        if length < self.mss then
            count = 1
        else
            count = floor((length + self.mss - 1) / self.mss)
        end

        if count > 255 then
            return -2
        end

        if 0 == count then
            count = 1
        end

        local offset = 0

        for i = 1, count do
            local size = 0
            if length - offset > self.mss then
                size = self.mss
            else
                size = length - offset
            end
            local seg = new_segment()
            copy_array(bytes, offset + index, seg.data, 0, size)

            offset = size + offset
            seg.frg = count - i + 1
            table.insert(self.snd_queue, seg)
        end
        return 0
    end,

    -- update ack.
    update_ack = function(self, rtt)
        if 0 == self.rx_srtt then
			self.rx_srtt = abs(rtt)
			self.rx_rttval = floor(self.rx_srtt / 2)
		else
            local delta = abs(abs(rtt) - self.rx_srtt)

			self.rx_rttval = floor((3 * self.rx_rttval + delta) / 4)
			self.rx_srtt = abs(floor((7 * self.rx_srtt + rtt) / 8))
			if self.rx_srtt < 1 then
				self.rx_srtt = 1
            end
        end

		local rto = self.rx_srtt + max(1, 4 * self.rx_rttval)
		self.rx_rto = bound(self.rx_minrto, abs(rto), IKCP_RTO_MAX)
    end,


    shrink_buf = function(self)
        if #self.snd_buf > 0 then
            self.snd_una = self.snd_buf[1].sn
        else
            self.snd_una = self.snd_nxt
        end
    end,

    parse_ack = function(self, sn)
        if sn - self.snd_una < 0 or sn - self.snd_nxt >= 0 then
            return
        end

        local index = 1
        for k,v in pairs(self.snd_buf) do
            if sn == v.sn then
                self.snd_buf = append(slice(self.snd_buf, 1, index - 1), slice(self.snd_buf, index + 1, #self.snd_buf))
                break
            end
            v.fastack = v.fastack + 1
            index = index + 1
        end
    end,

    parse_una = function(self, una)
		local count = 0
        for k,v in pairs(self.snd_buf) do
            if una - v.sn > 0 then
                count = count + 1
            else
                break
            end
        end

		if count > 0 then
            self.snd_buf = slice(self.snd_buf, count + 1, #self.snd_buf)
        end
	end,

    ack_push = function(self, sn, ts)
        table.insert(self.acklist, sn)
        table.insert(self.acklist, ts)
    end,

    ack_get = function(self, p)
        return self.acklist[p * 2 + 1], self.acklist[p * 2 + 2]
    end,

    parse_data = function(self, newseg)
		local sn = newseg.sn
        if sn - (self.rcv_nxt + self.rcv_wnd) >= 0 or sn - self.rcv_nxt < 0 then
            return
        end

		local n = #self.rcv_buf
		local after_idx = -1
		local rep = false

        local i = n
        while i >= 1 do
            local seg = self.rcv_buf[i]
            if seg.sn == sn then
				rep = true
				break
            end

			if sn - seg.sn > 0 then
                after_idx = i
				break
            end
            i = i - 1
        end

		if not rep then
            if after_idx == -1 then
				self.rcv_buf = append({ newseg }, self.rcv_buf)
			else
				self.rcv_buf = append(slice(self.rcv_buf, 1, after_idx),
				                      append({ newseg }, slice(self.rcv_buf, after_idx + 1, #self.rcv_buf)))
            end
        end

		-- move available data from rcv_buf -> rcv_queue
		local count = 0

        for k,v in pairs(self.rcv_buf) do
            if v.sn == self.rcv_nxt and #self.rcv_queue < self.rcv_wnd then
                table.insert(self.rcv_queue, v)
				self.rcv_nxt = self.rcv_nxt + 1
				count = count + 1
			else
				break
            end
        end

		if count > 0 then
            self.rcv_buf = slice(self.rcv_buf, count + 1, #self.rcv_buf)
        end
    end,

    -- when you received a low level packet (eg. UDP packet), call it
	Input = function(self, data)
		local s_una = self.snd_una
		if #data < IKCP_OVERHEAD then
			return -1
        end

		local offset = 0
		while true do
			local ts = 0
			local sn = 0
			local length = 0
			local una = 0
			local conv_ = 0

			local wnd = 0

			local cmd = 0
			local frg = 0

			if #data - offset < IKCP_OVERHEAD then
				break
            end

            local r
            r,conv_ = ikcp_decode32u(data, offset)
			offset = offset + r

			-- 这里我做了修改，不判断两端kcp conv相等，因为客户端也需要一个socket支持多个client连接
			--if (conv != conv_)
			--	return -1
            r,cmd = ikcp_decode8u(data, offset)
			offset = offset + r

            r,frg = ikcp_decode8u(data, offset)
			offset = offset + r

            r,wnd = ikcp_decode16u(data, offset)
			offset = offset + r

            r,ts = ikcp_decode32u(data, offset)
			offset = offset + r

            r,sn = ikcp_decode32u(data, offset)
			offset = offset + r

            r,una = ikcp_decode32u(data, offset)
            offset = offset + r

            r,length = ikcp_decode32u(data, offset)
            offset = offset + r

			if #data - offset < length then
				return -2
            end

            if cmd ~= IKCP_CMD_PUSH and cmd ~= IKCP_CMD_ACK and cmd ~= IKCP_CMD_WASK and cmd ~= IKCP_CMD_WINS then
                return -3
            end

			self.rmt_wnd = wnd
			self:parse_una(una)
			self:shrink_buf()

			if IKCP_CMD_ACK == cmd then
                if self.current - ts >= 0 then
                    self:update_ack(self.current - ts)
                end
				self:parse_ack(sn)
				self:shrink_buf()
			elseif IKCP_CMD_PUSH == cmd then
				if sn - (self.rcv_nxt + self.rcv_wnd) < 0 then
					self:ack_push(sn, ts)
					if sn - self.rcv_nxt >= 0 then
                        local seg = new_segment()
						seg.conv = conv_
						seg.cmd = cmd
						seg.frg = convert_index_to_0 and frg + 1 or frg
						seg.wnd = wnd
						seg.ts = ts
						seg.sn = sn
						seg.una = una

						if length > 0 then
                            copy_array(data, offset, seg.data, 0, length)
                        end
						self:parse_data(seg)
					end
				end
			elseif IKCP_CMD_WASK == cmd then
				-- ready to send back IKCP_CMD_WINS in Ikcp_flush
				-- tell remote my window size
				self.probe = self.probe | IKCP_ASK_TELL
			elseif IKCP_CMD_WINS == cmd then
				-- do nothing
			else
				return -3
            end

			offset = offset + length
		end

		if self.snd_una - s_una > 0 then
			if self.cwnd < self.rmt_wnd then
				local mss_ = self.mss
				if self.cwnd < self.ssthresh then
					self.cwnd = self.cwnd + 1
					self.incr = self.incr + mss_
				else

					if self.incr < mss_ then
						self.incr = mss_
                    end

					self.incr = self.incr + floor(mss_ * mss_ / self.incr + mss_ / 16)
					if (self.cwnd + 1) * mss_ <= self.incr then
						self.cwnd = self.cwnd + 1
                    end
				end
				if self.cwnd > self.rmt_wnd then
					self.cwnd = self.rmt_wnd
					self.incr = self.rmt_wnd * mss_
                end
            end
		end

		return 0
    end,

    wnd_unused = function(self)
		if #self.rcv_queue < self.rcv_wnd then
			return self.rcv_wnd - #self.rcv_queue
        end
		return 0
    end,

    -- flush pending data
	flush = function(self)
		local current_ = self.current
		local buffer_ = self.buffer
		local change = 0
		local lost = 0

		if 0 == self.updated then
			return
        end

        local seg = new_segment()
		seg.conv = self.conv
		seg.cmd = IKCP_CMD_ACK
		seg.wnd = self:wnd_unused()
		seg.una = self.rcv_nxt

		-- flush acknowledges
		local count = floor(#self.acklist / 2)
		local offset = 0

        for i = 1, count do
            if offset + IKCP_OVERHEAD > self.mtu then
                print("out put flush", offset)
				self.output(self.buffer, offset)
				offset = 0
            end

			seg.sn, seg.ts = self:ack_get(i - 1)
			offset = offset + seg:encode(self.buffer, offset)
        end
		self.acklist = {}

		-- probe window size (if remote window size equals zero)
		if 0 == self.rmt_wnd then
			if 0 == self.probe_wait then
				self.probe_wait = IKCP_PROBE_INIT
				self.ts_probe = self.current + self.probe_wait
			elseif self.current - self.ts_probe >= 0 then
				if self.probe_wait < IKCP_PROBE_INIT then
					self.probe_wait = IKCP_PROBE_INIT
                end

				self.probe_wait = self.probe_wait + floor(self.probe_wait / 2)
				if self.probe_wait > IKCP_PROBE_LIMIT then
					self.probe_wait = IKCP_PROBE_LIMIT
                end

				self.ts_probe = self.current + self.probe_wait
				self.probe = self.probe | IKCP_ASK_SEND
            end
		else
			self.ts_probe = 0
			self.probe_wait = 0
        end

		-- flush window probing commands
		if (self.probe & IKCP_ASK_SEND) ~= 0 then
			seg.cmd = IKCP_CMD_WASK
			if offset + IKCP_OVERHEAD > self.mtu then
				self.output(self.buffer, offset)
				offset = 0
            end
			offset = offset + seg:encode(self.buffer, offset)
        end

		self.probe = 0

		-- calculate window size
		local cwnd_ = min(self.snd_wnd, self.rmt_wnd)
		if 0 == self.nocwnd then
			cwnd_ = min(self.cwnd, cwnd_)
        end

		count = 0
		for k = 1, #self.snd_queue do
			if self.snd_nxt - (self.snd_una + cwnd_)  >= 0 then
				break
            end

			local newseg = self.snd_queue[k]
			newseg.conv = self.conv
			newseg.cmd = IKCP_CMD_PUSH
			newseg.wnd = seg.wnd
			newseg.ts = current_
			newseg.sn = self.snd_nxt
			newseg.una = self.rcv_nxt
			newseg.resendts = current_
			newseg.rto = self.rx_rto
			newseg.fastack = 0
			newseg.xmit = 0

            table.insert(self.snd_buf, newseg)
			self.snd_nxt = self.snd_nxt + 1
			count = count + 1
		end

		if count > 0 then
			self.snd_queue = slice(self.snd_queue, count + 1, #self.snd_queue)
        end

		-- calculate resent
		local resent = self.fastresend
		if self.fastresend <= 0 then
			resent = 0xffffffff
        end

		local rtomin = self.rx_rto >> 3
		if self.nodelay ~= 0 then
			rtomin = 0
        end

		-- flush data segments
        for k,segment in pairs(self.snd_buf) do
			local needsend = false
			local debug = current_ - segment.resendts
			if 0 == segment.xmit then
				needsend = true
				segment.xmit = segment.xmit + 1
				segment.rto = self.rx_rto
				segment.resendts = current_ + segment.rto + rtomin
			elseif current_ - segment.resendts >= 0 then
				needsend = true
                segment.xmit = segment.xmit + 1
				self.xmit = self.xmit + 1
				if 0 == self.nodelay then
					segment.rto = segment.rto + self.rx_rto
				else
					segment.rto = segment.rto + floor(self.rx_rto / 2)
                end

				segment.resendts = current_ + segment.rto
				lost = 1
			elseif segment.fastack >= resent then
				needsend = true
                segment.xmit = segment.xmit + 1
				segment.fastack = 0
				segment.resendts = current_ + segment.rto
                change = change + 1
            end

			if needsend then
				segment.ts = current_
				segment.wnd = seg.wnd
				segment.una = self.rcv_nxt

				local need = IKCP_OVERHEAD + #segment.data
				if offset + need > self.mtu then
					self.output(self.buffer, offset)
					offset = 0
                end

				offset = offset + segment:encode(self.buffer, offset)
				if #segment.data > 0 then
                    copy_array(segment.data, 0, self.buffer, offset, #segment.data)
					offset = offset + #segment.data
                end

				if segment.xmit >= self.dead_link then
					self.state = 0
                end
            end
		end

		-- flash remain segments
		if offset > 0 then
			self.output(self.buffer, offset)
			offset = 0
        end

		-- update ssthresh
		if change ~= 0 then
			local inflight = self.snd_nxt - self.snd_una
			self.ssthresh = floor(inflight / 2)
			if self.ssthresh < IKCP_THRESH_MIN then
				self.ssthresh = IKCP_THRESH_MIN
            end

			self.cwnd = self.ssthresh + resent
			self.incr = self.cwnd * self.mss
		end

		if lost ~= 0 then
			self.ssthresh = floor(self.cwnd / 2)
			if self.ssthresh < IKCP_THRESH_MIN then
				self.ssthresh = IKCP_THRESH_MIN
            end

			self.cwnd = 1
			self.incr = self.mss
		end

		if self.cwnd < 1 then
            self.cwnd = 1
			self.incr = self.mss
        end
	end,

    -- update state (call it repeatedly, every 10ms-100ms), or you can ask
    -- ikcp_check when to call it again (without ikcp_input/_send calling).
    -- 'current' - current timestamp in millisec.
    Update = function(self, current_)
        self.current = current_

        if 0 == self.updated then
            self.updated = 1
            self.ts_flush = self.current
        end

        local slap = self.current - self.ts_flush
        if slap >= 10000 or slap < -10000 then
            self.ts_flush = self.current
            slap = 0
        end

        if slap >= 0 then
            self.ts_flush = self.ts_flush + self.interval
            if self.current - self.ts_flush >= 0 then
                self.ts_flush = self.current + self.interval
            end
            self:flush()
        end
    end,

    -- fastest: ikcp_nodelay(kcp, 1, 20, 2, 1)
	-- nodelay: 0:disable(default), 1:enable
	-- interval: internal update timer interval in millisec, default is 100ms
	-- resend: 0:disable fast resend(default), 1:enable fast resend
	-- nc: 0:normal congestion control(default), 1:disable congestion control
	NoDelay = function(self, nodelay_, interval_, resend_, nc_)
		if nodelay_ > 0 then
			self.nodelay = nodelay_
			self.rx_minrto = IKCP_RTO_NDL
        end

		if interval_ >= 0 then
			if interval_ > 5000 then
				interval_ = 5000
			elseif interval_ < 10 then
				interval_ = 10
            end
			interval = interval_
		end

		if resend_ >= 0 then
			self.fastresend = resend_
        end

		if nc_ >= 0 then
			self.nocwnd = nc_
        end

		return 0
	end,

    -- set maximum window size: sndwnd=32, rcvwnd=32 by default
    WndSize = function(self, snd_wnd_, rcv_wnd_)
        if snd_wnd_ > 0 then
            self.snd_wnd = snd_wnd_
        end

        if rcv_wnd_ > 0 then
            self.rcv_wnd = rcv_wnd_
        end
        return 0
    end,

    WaitSnd = function(self)
		return #self.snd_buf + #self.snd_queue;
    end,
}

	-- Determine when should you invoke ikcp_update:
	-- returns when you should invoke ikcp_update in millisec, if there
	-- is no ikcp_input/_send calling. you can call ikcp_update in that
	-- time, instead of call update repeatly.
	-- Important to reduce unnacessary ikcp_update invoking. use it to
	-- schedule ikcp_update (eg. implementing an epoll-like mechanism,
	-- or optimize ikcp_update when handling massive kcp connections)
	-- public UInt32 Check(UInt32 current_)
	-- {
	-- 	if (0 == updated)
	-- 	{
	-- 		return current_
	-- 	}
    --
	-- 	uint ts_flush_ = ts_flush
	-- 	int tm_flush_ = 0x7fffffff
	-- 	int tm_packet = 0x7fffffff
	-- 	int minimal = 0
    --
	-- 	if (_itimediff(current_, ts_flush_) >= 10000 || _itimediff(current_, ts_flush_) < -10000)
	-- 	{
	-- 		ts_flush_ = current_
	-- 	}
    --
	-- 	if (_itimediff(current_, ts_flush_) >= 0)
	-- 	{
	-- 		return current_
	-- 	}
    --
	-- 	tm_flush_ = _itimediff(ts_flush_, current_)
    --
	-- 	foreach (Segment seg in snd_buf)
	-- 	{
	-- 		int diff = _itimediff(seg.resendts, current_)
	-- 		if (diff <= 0)
	-- 		{
	-- 			return current_
	-- 		}
    --
	-- 		if (diff < tm_packet)
	-- 		{
	-- 			tm_packet = diff
	-- 		}
	-- 	}
    --
	-- 	minimal = tm_packet
	-- 	if (tm_packet >= tm_flush_)
	-- 	{
	-- 		minimal = tm_flush_
	-- 	}
    --
	-- 	if (minimal >= interval)
	-- 	{
	-- 		minimal = (int)this.interval
	-- 	}
    --
	-- 	return current_ + (UInt32)minimal
	-- }

	-- change MTU size, default is 1400
	-- public int SetMtu(Int32 mtu_)
	-- {
	-- 	if (mtu_ < 50 || mtu_ < IKCP_OVERHEAD)
	-- 	{
	-- 		return -1
	-- 	}
    --
	-- 	var buffer_ = new byte[(mtu_ + IKCP_OVERHEAD) * 3]
	-- 	if (null == buffer_)
	-- 	{
	-- 		return -2
	-- 	}
    --
	-- 	mtu = (UInt32)mtu_
	-- 	mss = mtu - IKCP_OVERHEAD
	-- 	buffer = buffer_
	-- 	return 0
	-- }

	-- public int Interval(Int32 interval_)
	-- {
	-- 	if (interval_ > 5000)
	-- 	{
	-- 		interval_ = 5000
	-- 	}
	-- 	else if (interval_ < 10)
	-- 	{
	-- 		interval_ = 10
	-- 	}
    --
	-- 	interval = (UInt32)interval_
	-- 	return 0
	-- }

	-- get how many packet is waiting to be sent
	-- public int WaitSnd()
	-- {
	-- 	return snd_buf.Length + snd_queue.Length
	-- }

    return function(conv_, output_)
        local t = clone(kcp)
        t.conv = conv_
		t.snd_wnd = IKCP_WND_SND
		t.rcv_wnd = IKCP_WND_RCV
		t.rmt_wnd = IKCP_WND_RCV
		t.mtu = IKCP_MTU_DEF
		t.mss = t.mtu - IKCP_OVERHEAD

		t.rx_rto = IKCP_RTO_DEF
		t.rx_minrto = IKCP_RTO_MIN
		t.interval = IKCP_INTERVAL
		t.ts_flush = IKCP_INTERVAL
		t.ssthresh = IKCP_THRESH_INIT
		t.dead_link = IKCP_DEADLINK
		t.buffer = {}
		t.output = output_
        return t
    end
