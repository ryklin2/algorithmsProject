[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/fzjqrJB1)
# Algorithms Project: Optimal Routing for Self-Driving Cars

## Student Information

Please complete this section before submitting:

- Name(s):  
  - Daniel Omstead

- Email(s):  
  - domstead@sfsu.edu

---

## Project Overview

In this project, your team will collaborate to design and implement an algorithm that addresses a real-world problem faced by companies like Waymo: determining the most profitable and cost-effective route for a self-driving car to take a passenger from their pickup location to their destination.

The challenge involves considering multiple real-world constraints that affect route profitability and customer cost. You'll explore both naive and optimized solutions, analyze time/space trade-offs, and write actual code with unit tests.

You may work solo or in teams of up to 3 students.

---

## Problem Statement

You are a software engineer for a company that develops self-driving cars. Your current task is to build an algorithm that:

- Finds the optimal tour from pickup to destination.
- Maximizes company profit by minimizing the frequency of car recharging (electric cars).
- Minimizes cost for the customer using dynamic pricing.

The algorithm should consider the following constraints:

1. Traffic Volume: More traffic means higher electricity usage, reducing company profit.
2. Demand-Based Pricing: More demand → lower prices (inverse of surge pricing).
3. Vehicle Availability: Fewer available cars can impact route selection.

---

## Your Tasks

Your final submission should include the following components:

### 1. Brainstorming & Idea Development
- Use plain English, diagrams, or flowcharts.
- Describe your team's thinking process and key insights.

### 2. Brute Force Approach
- Provide a naive solution to the problem.
- Include explanation and potential algorithm.

### 3. Complexity Trade-offs
- Analyze time and space complexity of your brute-force solution.
- Discuss where it performs well or poorly.

### 4. Optimized Algorithm
- Develop an efficient algorithm that improves on the brute-force version.
- Consider algorithmic strategies like Greedy, Dynamic Programming, Graph Algorithms, Branch & Bound Approaches, A*, and more.

### 5. Pseudocode
- Write pseudocode for your optimized algorithm.

### 6. Complexity & Trade-Off Analysis
- Compute the worst-case time and space complexity of your optimized algorithm.
- Discuss how it compares to the brute-force approach.

### 7. Code Implementation
- Write the optimized algorithm in your favorite programming language (Python, Java, etc.).
- Ensure code is clean and documented.

### 8. Unit Testing
- Add unit tests to verify the correctness of your algorithm.
- Include edge cases (e.g., no available cars, heavy traffic).

### 9. Team Contributions
- If you're working in a team, include a section where each member briefly explains their contributions.
- Solo students can skip this section.

---

## Deliverables (must be included in this repository at submission time)

- A single PDF or Markdown report with:
  - Descriptions, diagrams, pseudocode, and analysis.
- An executable file with your source code, including unit tests. You can use your favorite programming language for this part of the project.

## Submission on Canvas

- Submit the link to your GitHub repository using the submission link provided on Canvas.  
- If your repository link is not submitted by the deadline, we will not be able to locate your work, and it will be considered not submitted, resulting in a grade of zero.

---

## Grading Rubric (15 Points Total)

| Category                             | Points | Description |
|--------------------------------------|--------|-------------|
| Brainstorming & Problem Understanding| 2      | Clear explanation of problem and initial ideas. |
| Brute-Force Solution + Analysis      | 2      | Reasonable naive solution with complexity discussion. |
| Optimized Algorithm + Pseudocode     | 3      | Efficient and thoughtful algorithm with clear pseudocode. |
| Time & Space Complexity Analysis     | 2      | Accurate analysis and thoughtful trade-off discussion. |
| Code Implementation                  | 2      | Functional and clean code implementing optimized solution. |
| Unit Testing                         | 2      | Includes meaningful unit tests covering edge cases. |
| Team Contributions (if in team)      | 1      | Clear breakdown of roles (solo students get full point). |
| Total                                | 15     |  |

---

## Tips for Success

- Start with brainstorming and research real-world applications of traffic-aware routing.
- Think in terms of graphs, shortest paths, or dynamic weights.
- Test your algorithm under different simulated scenarios.
- Do not use external libraries for this project; it will reinforce your understanding of the problem.

---

## Due Date

Monday, May 5 at 2:00 PM. Please plan your time accordingly, as no extensions will be granted for this project.

---

If you have any questions, feel free to ask during class, office hours or post in the course Discord channel.

Good luck, everyone!
