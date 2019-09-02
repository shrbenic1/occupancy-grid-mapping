function logOdds = probabilityToLogOdds (probability)
Ones = ones(size(probability, 1), size(probability, 2));
logOdds = log10(probability./(Ones - probability));
end