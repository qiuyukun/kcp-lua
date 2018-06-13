# kcp-lua
基于lua5.3版本实现的kcp
如果为lua5.3以前的版本,可以使用bit库替换位运算操作

# Notice
如果远端的kcp使用的语言类型的下标是从0开始的

则需要开启
convert_index_to_0 = true

如果也是像lua一样顺序表从1开始,则设置为false

# API

创建
<pre><code>
local ikcp_create = require("ikcp")

local kcp1 = ikcp_create(1, function(buf, len)
    --当有消息需要发送时会回调
    --buf为顺序表,value是number,需转换成字节发送
    --len需要发送的长度
end)

</code></pre>

配置
<pre><code>
    --急速模式
    kcp1:NoDelay(1, 10, 2, 1)
    --窗口大小
    kcp1:WndSize(256, 256)
    --default is 1400
    kcp1:SetMtu(1400)
    --设置更新间隔
    kcp1:SetInterval()
</code></pre>

Check
<pre><code>
    --当前时间戳
    kcp1:Check(timestamp)
</code></pre>

Update
<pre><code>
    --当前时间戳
    kcp1:Update(timestamp)
</code></pre>

Input
<pre><code>
    --当收到socket消息后,传入一个包含字节信息的table
    local bytes = {str:byte(1, -1)}
    kcp1:Input(bytes)
</code></pre>

Send
<pre><code>
    --发送消息,传入一个包含字节信息的table
    local bytes = { str:byte(1, -1) }
    kcp1:Send(bytes, 0, #bytes)
</code></pre>

Recv
<pre><code>
    --接受消息,传入一个空table
    --返回长度,如果大于0则表示有消息,buf里会填充字节信息
    local buf = {}
    local len = kcp1:Recv(buf)
</code></pre>

-----------------

kcp详细说明以及其它版本

https://github.com/skywind3000/kcp.git
