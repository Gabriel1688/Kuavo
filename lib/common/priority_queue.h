#pragma  once

#include <algorithm>
#include <concepts>
#include <functional>
#include <utility>
#include <vector>


/**
 * This class is the same as std::priority_queue with two changes:
 *
 * 1. Adds a remove() function for removing all elements from the priority queue
 *    that match the given value.
 * 2. Replaces "void pop()" with "T pop()" so the element can be moved from the
 *    queue directly instead of copied from top().
 */
template<typename T, typename Sequence = std::vector<T>,
        typename Compare = std::less<typename Sequence::value_type>> requires std::same_as<T, typename Sequence::value_type>
class priority_queue {
public:
    using value_type = typename Sequence::value_type;
    using reference = typename Sequence::reference;
    using const_reference = typename Sequence::const_reference;
    using size_type = typename Sequence::size_type;
    using container_type = Sequence;
    using value_compare = Compare;

    template<typename Seq = Sequence>
    requires std::default_initializable<Compare> &&
             std::default_initializable<Seq>
    priority_queue() {}

    priority_queue(const Compare &comp, const Sequence &c) : m_c(c), m_comp(comp) {
        std::make_heap(m_c.begin(), m_c.end(), m_comp);
    }

    explicit priority_queue(const Compare &comp, Sequence &&c = Sequence{})
            : m_c(std::move(c)), m_comp(comp) {
        std::make_heap(m_c.begin(), m_c.end(), m_comp);
    }

    template<typename InputIterator>
    priority_queue(InputIterator first, InputIterator last, const Compare &comp,
                   const Sequence &c)
            : m_c(c), m_comp(comp) {
        m_c.insert(m_c.end(), first, last);
        std::make_heap(m_c.begin(), m_c.end(), m_comp);
    }

    template<typename InputIterator>
    priority_queue(InputIterator first, InputIterator last,
                   const Compare &comp = Compare{}, Sequence &&c = Sequence{})
            : m_c(std::move(c)), m_comp(comp) {
        m_c.insert(m_c.end(), first, last);
        std::make_heap(m_c.begin(), m_c.end(), m_comp);
    }

    [[nodiscard]]
    bool empty() const {
        return m_c.empty();
    }

    size_type size() const { return m_c.size(); }

    const_reference top() const { return m_c.front(); }

    void push(const value_type &value) {
        m_c.push_back(value);
        std::push_heap(m_c.begin(), m_c.end(), m_comp);
    }

    void push(value_type &&value) {
        m_c.push_back(std::move(value));
        std::push_heap(m_c.begin(), m_c.end(), m_comp);
    }

    template<typename... Args>
    void emplace(Args &&... args) {
        m_c.emplace_back(std::forward<Args>(args)...);
        std::push_heap(m_c.begin(), m_c.end(), m_comp);
    }

    T pop() {
        std::pop_heap(m_c.begin(), m_c.end(), m_comp);
        auto ret = std::move(m_c.back());
        m_c.pop_back();
        return ret;
    }

    bool remove(const T &value) {
        auto it = std::find(m_c.begin(), m_c.end(), value);
        if (it != this->m_c.end()) {
            m_c.erase(it);
            std::make_heap(m_c.begin(), m_c.end(), m_comp);
            return true;
        } else {
            return false;
        }
    }

protected:
    Sequence m_c;
    Compare m_comp;
};