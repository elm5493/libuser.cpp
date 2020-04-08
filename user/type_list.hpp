#ifndef __TYPE_LIST_HPP__
#define __TYPE_LIST_HPP__

#include <cstddef>
#include <type_traits>



namespace user {

namespace type_list {


template <class ...List> class type_list;


template <class ...List>
class type_list {

  private:

	template <std::size_t N>
	using _size_t = std::integral_constant<std::size_t, N>;

	template <class I, class ...L>          struct _at_type                      {};
	template <class I, class T, class ...L> struct _at_type<I, T, L...>          { using type = typename _at_type<_size_t<I::value-1>, L...>::type; };
	template <class I>                      struct _at_type<I>                   { using type = void; };
	template <class T, class ...L>          struct _at_type<_size_t<0>, T, L...> { using type = T; };

	template <class I, class ...L>          struct _erase_front_type                      {};
	template <class I, class T, class ...L> struct _erase_front_type<I, T, L...>          { using type = typename _erase_front_type<_size_t<I::value-1>, L...>::type; };
	template <class ...L>                   struct _erase_front_type<_size_t<0>, L...>    { using type = type_list<L...>; };
	template <class T, class ...L>          struct _erase_front_type<_size_t<1>, T, L...> { using type = type_list<L...>; };
	template <class I>                      struct _erase_front_type<I>                   { using type = type_list<>; };

	template <class I, class ...L>          struct _extract_front_type                      {};
	template <class I, class T, class ...L> struct _extract_front_type<I, T, L...>          { using type = typename _extract_front_type<_size_t<I::value-1>, L...>::type::template prepend<T>; };
	template <class T, class ...L>          struct _extract_front_type<_size_t<1>, T, L...> { using type = type_list<T>; };
	template <class ...L>                   struct _extract_front_type<_size_t<0>, L...>    { using type = type_list<>; };
	template <class I>                      struct _extract_front_type<I>                   { using type = type_list<>; };

	// _concat_type
	template <class ...L> struct _concat_type {};
	template <class ...L, class ...R> struct _concat_type< type_list<L...>, R... > {
		using type = typename _concat_type< type_list<L...>, typename _concat_type<R...>::type >::type;
	};
	template <class ...L1, class ...L2> struct _concat_type< type_list<L1...>, type_list<L2...> > {
		using type = type_list<L1..., L2...>;
	};

	// _insert_type
	template <class I, class L, class ...Add> struct _insert_type {
		using type = typename _insert_type<_size_t<I-1>, typename L::template erase<0>, Add...>::type::template prepend<typename L::first>;
	};
	template <class L, class ...Add> struct _insert_type<_size_t<0>, L, Add...> {
		using type = typename L::template prepend<Add...>;
	};
	template <class I, class ...Add> struct _insert_type<I, type_list<>, Add...> {
		using type = type_list<Add...>;
	};
	template <class L, class ...Add> struct _insert_type<_size_t<sizeof...(List)>, L, Add...> {
		using type = typename L::template append<Add...>;
	};


	// _erase_type
	template <class I, class ...L> struct _erase_type {};
	template <class I, class T, class ...L> struct _erase_type<I, T, L...> {
		using type = typename _erase_type<_size_t<I::value-1>, L...>::type::template prepend<T>;
	};
	template <class T, class ...L> struct _erase_type<_size_t<0>, T, L...> {
		using type = type_list<L...>;
	};
//	template <std::size_t I, class T> struct _erase_type<I, T> {
//		using type = type_list<>;
//	};
	template <class I> struct _erase_type<I> {
		using type = type_list<>;
	};


	// _set_type
	template <class N, class T> struct _set_type {
		using type = typename _concat_type<
			typename _extract_front_type<_size_t<N::value>, List...>::type::template append<T>,
			typename _erase_front_type<_size_t<N::value+1>, List...>::type
		>::type;
	};
	template <class T> struct _set_type<_size_t<0>, T> {
		using type = typename _erase_front_type<_size_t<1>, List...>::type::template prepend<T>;
	};
	template <class T> struct _set_type<_size_t<sizeof...(List)-1>, T> {
		using type = typename _extract_front_type<_size_t<sizeof...(List)-1>, List...>::type::template append<T>;
	};

  public:

	static constexpr const std::size_t size = sizeof...(List);

	template <std::size_t N>
	using at = typename _at_type<_size_t<N>, List...>::type;

	using first = typename _at_type<_size_t<0>, List...>::type;
	using last  = typename _at_type<_size_t<size-1>, List...>::type;


	template <std::size_t N>
	using front = at<N>;

	template <std::size_t N>
	using back = at<size - N - 1>;

	template <class ...Add>
	using append = type_list<List..., Add...>;

	template <class ...Add>
	using prepend = type_list<Add..., List...>;

	template <std::size_t N>
	using extract_front = typename _extract_front_type<_size_t<N>, List...>::type;

	template <std::size_t N>
	using extract_back = typename _erase_front_type<_size_t<(size > N) ? (size - N) : (0)>, List...>::type;

	template <std::size_t N>
	using erase_front = typename _erase_front_type<_size_t<N>, List...>::type;

	template <std::size_t N>
	using erase_back = typename _extract_front_type<_size_t<(size > N) ? (size - N) : (0)>, List...>::type;

	template <class ...L>
	using concat = typename _concat_type< type_list<List...>, L...>::type;

	template <std::size_t N, class ...Add>
	using insert = typename _insert_type<_size_t<(N > size) ? (size) : (N)>, type_list<List...>, Add...>::type;

	template <std::size_t N>
	using erase = typename _erase_type<_size_t<N>, List...>::type;

	template <std::size_t N, class T>
	using set = typename _set_type<_size_t<(N < size) ? (N) : (size - 1)>, T>::type;

	template <class T>
	using push_front = prepend<T>;
	using pop_front = erase<0>;

	template <template <class ...> class T>
	using apply = T<List...>;

	template <template <class> class F>
	using map = type_list< typename F<List>::type... >;

};

template <class List1, class List2>
struct concat_type_list {};

template <class L, class ...Rest>
struct concat_type_list< L, type_list<Rest...> > {
	using type = typename L::template concat<Rest...>;
};


template <std::size_t N, class T> struct replicate       { using type = typename replicate<N-1, T>::type::template append<T>; };
template <class T>                struct replicate<0, T> { using type = type_list<>; };


template <class L, template <class ...> class T> struct apply;
template <class ...L, template <class ...> class T> struct apply< type_list<L...>, T > {
	using type = T<L...>;
};


template <class F>
class arguments_type_list {

  private:

	template <class T, bool is_func> class _get_args;

	template <class T> class _get_args<T, true> {
	  private:
		template <class R, class ...Args>
		static auto _impl(R(*)(Args...)) -> type_list<Args...>;
	  public:
		using type = decltype( _impl(std::declval<T>()) );
	};

	template <class T> class _get_args<T, false> {
	  private:
		template <class R, class C, class ...Args>
		static auto _impl(R(C::*)(Args...) const) -> type_list<Args...>;
	  public:
		using type = decltype( _impl(&T::operator()) );
	};

  public:
	using type = typename _get_args<F, std::is_function< typename std::remove_pointer<F>::type >::value>::type;

};

template <class F>
using arguments_type_list_t = typename arguments_type_list<F>::type;


} // namespace type_list


} // namespace user

#endif // !__TYPE_LIST_HPP__


