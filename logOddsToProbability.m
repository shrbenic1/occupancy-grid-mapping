function probability = logOddsToProbability (logOdds)
Ones = ones(size(logOdds, 1), size(logOdds, 2));
probability = Ones - Ones./(Ones + 10.^(logOdds));
end