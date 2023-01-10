clc
clear all

ticketMap = containers.Map(...
      {'jan', 'feb', 'march'}, ...
      {10, 20, 30});
%%Retrieve specified Key wrt defined Value using CELLFUN

d = Node([7 8])
e = Node([10 7])

d.f = 30
e.f = 51

visited = containers.Map

visited('1') = d
visited('2') = e

container = visited
value = cellfun(@getNum, values(container))
% 
[minimum, index] = min(value)

keys = keys(container)

lowest = container(keys{1, index})

function num = getNum(element)
   num = element.f;
end


% 
% 
% testvalue = 30;
% testind = cellfun(@min,values(ticketMap));
% testkeys = keys(ticketMap);
% msg_key = testkeys(testind)