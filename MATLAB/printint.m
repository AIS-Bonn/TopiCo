function printint(number) %#codegen

    if (number < 0)
        fprintf('-');
    end
    
    number = uint64(abs(number));
    b_leading = true;
    divisor = uint64(10000000000000000000);
    while (divisor >= 1)
        d = idivide(number,divisor);
        if (d ~= 0)
            b_leading = false;
        end
        if (b_leading == false)
            number = number - d * divisor;
            switch d
                case 0
                    fprintf('0');
                case 1
                    fprintf('1');
                case 2
                    fprintf('2');
                case 3
                    fprintf('3');
                case 4
                    fprintf('4');
                case 5
                    fprintf('5');
                case 6
                    fprintf('6');
                case 7
                    fprintf('7');
                case 8
                    fprintf('8');
                case 9
                    fprintf('9');
            end
        end
        divisor = divisor / 10;
    end
    if (b_leading == true)
        fprintf('0');
    end

end
