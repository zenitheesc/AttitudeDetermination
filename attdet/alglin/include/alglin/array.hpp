#ifndef ARRAY_HPP
#define ARRAY_HPP

#if __cplusplus >= 201703L
#define CONSTEXPR_17 constexpr
#else
#define CONSTEXPR_17
#endif


namespace alglin {
template<class Type, int N> class array {

	struct iterator {
		/* not "real" iterator */
		// using iterator_category = std::forward_iterator_tag;

		constexpr iterator(Type *ptr) : m_ptr(ptr) {}

		constexpr Type &operator*() const { return *m_ptr; }
		Type *operator->() { return m_ptr; }
		iterator &operator++() {
			m_ptr++;
			return *this;
		}
		iterator operator++(int) {
			iterator tmp = *this;
			++(*this);
			return tmp;
		}
		constexpr friend bool operator==(const iterator &a, const iterator &b) {
			return a.m_ptr == b.m_ptr;
		};
		constexpr friend bool operator!=(const iterator &a, const iterator &b) {
			return a.m_ptr != b.m_ptr;
		}

	  private:
		Type *m_ptr;
	};

  private:
	Type elem[N];

  public:
	constexpr array() = default;
	array(Type from[]) : elem{ from } {}
	constexpr Type operator[](int i) const { return elem[i]; }
	Type &operator[](int i) { return elem[i]; }
	Type *data() { return elem; }
	iterator begin() { return iterator(&elem[0]); }
	iterator end() { return iterator(&elem[N]); }// one pass the end
};
}// namespace alglin
#undef CONSTEXPR_17
#endif
