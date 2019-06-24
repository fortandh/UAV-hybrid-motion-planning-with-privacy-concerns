# algorithm for the quicksort


def quick_sort(the_list):
    """快速排序主函数"""
    quick_sort_helper(the_list, 0, len(the_list) - 1)


def quick_sort_helper(the_list, first, last):
    if first < last:
        split_value = partition(the_list, first, last)

        quick_sort_helper(the_list, first, split_value - 1)
        quick_sort_helper(the_list, split_value + 1, last)


def partition(the_list, first, last):
    pivot_value = the_list[first].fitness

    left_mark = first + 1
    right_mark = last

    done = False
    while not done:
        while left_mark <= right_mark and the_list[left_mark].fitness >= pivot_value:
            left_mark += 1

        while left_mark <= right_mark and the_list[right_mark].fitness <= pivot_value:
            right_mark -= 1

        if left_mark > right_mark:
            done = True
        else:
            tmp = the_list[left_mark]
            the_list[left_mark] = the_list[right_mark]
            the_list[right_mark] = tmp

    tmp = the_list[first]
    the_list[first] = the_list[right_mark]
    the_list[right_mark] = tmp
    return right_mark
